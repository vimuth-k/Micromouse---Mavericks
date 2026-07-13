/**
 * @file    encoders.c
 * @brief   Quadrature encoder reading — implementation.
 *
 * @details WHY TWO DIFFERENT TIMERS
 *          ──────────────────────────
 *          The STM32F411 has one 32-bit general-purpose timer (TIM2) and
 *          several 16-bit timers (TIM3, TIM4, TIM5).  The left encoder
 *          uses TIM2 (32-bit) so its counter never wraps during any
 *          realistic maze run — no overflow handling is needed at all.
 *
 *          The right encoder uses TIM4 (16-bit) because TIM2 is already
 *          taken.  A 16-bit counter wraps at 65535.  At SPD_SEARCH the
 *          robot travels ~1.57 counts/ms; it would take ~42 seconds of
 *          continuous forward motion to wrap once.  During a turn the
 *          wrap can happen much faster.  The overflow ISR hook
 *          (encoders_tim4_ovf_irq) tracks every wrap to extend the
 *          effective range to 32 bits.
 *
 *          OVERFLOW DIRECTION DETECTION
 *          ─────────────────────────────
 *          When TIM4 wraps, its update interrupt fires.  At that moment
 *          TIM4->CNT has already rolled over.  We determine direction by
 *          comparing the new CNT value against the midpoint (0x8000):
 *            CNT ≈ 0      → counter wrapped from 65535 to 0 → forward
 *            CNT ≈ 65535  → counter wrapped from 0 to 65535 → reverse
 *          The overflow counter is incremented for forward wraps and
 *          decremented for reverse wraps.
 *
 *          32-BIT RIGHT COUNT ASSEMBLY
 *          ────────────────────────────
 *          right_count = (overflow × 65536) + TIM4->CNT
 *          Example: 2 forward overflows, CNT = 12340
 *            = (2 × 65536) + 12340 = 143412 counts
 *          Example: 1 reverse underflow, CNT = 63210
 *            = (-1 × 65536) + 63210 = -2326 counts
 *
 *          RACE CONDITION MITIGATION
 *          ──────────────────────────
 *          Between reading TIM4->CNT and reading s_right_overflow, the
 *          TIM4 IRQ could fire and increment the overflow counter while
 *          CNT is still near 65535 (just before it wraps).  This gives
 *          a transiently incorrect reading.  We detect this by reading
 *          CNT a second time: if both readings agree, no wrap occurred
 *          during the read.  If they differ, we retry once.  This costs
 *          two extra register reads in the worst case — acceptable in a
 *          1 ms control loop.
 *
 *          DELTA TRACKING
 *          ───────────────
 *          enc_left_delta() and enc_right_delta() cache the previous
 *          count in static variables.  The delta is (current − previous).
 *          The PID in motion.c calls these exactly once per 1 ms tick.
 *          The returned delta feeds:
 *            - Speed PID error:      target_speed − delta × MM_PER_COUNT × 1000
 *            - Straightness PID:     cumulative count_left − count_right
 *
 * @author  VDawn
 * @date    2026
 */

#include "encoders.h"
#include "pins.h"
#include "config.h"
#include "main.h"     /* htim2, htim4 externs */

/* =========================================================================
 * PRIVATE STATE
 * ======================================================================= */

/** TIM4 16-bit overflow/underflow extension counter.
 *  Incremented on forward wrap (65535→0), decremented on reverse (0→65535).
 *  Declared volatile because it is written in an ISR and read in main code. */
static volatile int32_t  s_right_overflow = 0;

/** Last TIM4 CNT value observed in the overflow ISR.
 *  Used to determine wrap direction (forward vs reverse). */
static volatile uint16_t s_right_last_cnt = 0U;

/** Previous counts for delta calculation — updated by enc_XX_delta(). */
static int32_t s_left_prev  = 0;
static int32_t s_right_prev = 0;

/** Most recent deltas — cached for speed estimation by enc_XX_speed_mmps(). */
static int32_t s_left_delta_last  = 0;
static int32_t s_right_delta_last = 0;

/** True after encoders_init() has been called. */
static bool s_initialised = false;

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Read TIM4 CNT combined with overflow counter into a 32-bit value.
 *
 * @details Implements the double-read anti-race technique:
 *          1. Read TIM4->CNT (first read).
 *          2. Read s_right_overflow.
 *          3. Read TIM4->CNT again (second read).
 *          If both CNT reads match, no wrap occurred and the combination
 *          is valid.  If they differ, re-read the overflow counter and
 *          use the second CNT value (the wrap has already completed).
 *
 *          RIGHT_ENC_POL is applied here so all callers get a
 *          sign-corrected value.
 *
 * @return int32_t  Signed 32-bit right encoder count.
 */
static int32_t read_right_count(void)
{
    uint16_t cnt_a;
    uint16_t cnt_b;
    int32_t  ovf;

    cnt_a = ENC_R_CNT;                    /* first read  */
    ovf   = s_right_overflow;             /* volatile read */
    cnt_b = ENC_R_CNT;                    /* second read */

    if (cnt_b != cnt_a)
    {
        /* A wrap occurred between the two reads — re-read overflow.
         * The wrap is now complete so this read is consistent.       */
        ovf   = s_right_overflow;
        cnt_a = cnt_b;
    }

    int32_t raw = (ovf * 65536) + (int32_t)cnt_a;
    return raw * RIGHT_ENC_POL;
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Start both encoder timers and zero all state.
 */
MmResult_t encoders_init(void)
{
    /* TIM2 and TIM4 must already be configured in encoder mode by main.c */
    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* Zero hardware counters */
    ENC_RESET_ALL();

    /* Zero all software state */
    s_right_overflow   = 0;
    s_right_last_cnt   = 0U;
    s_left_prev        = 0;
    s_right_prev       = 0;
    s_left_delta_last  = 0;
    s_right_delta_last = 0;
    s_initialised      = true;

    return MM_OK;
}

/**
 * @brief  Zero both hardware counters and all accumulated state.
 *
 * @details Called before every move to make enc_avg_mm() measure
 *          distance-from-here rather than distance-from-boot.
 */
void encoders_reset(void)
{
    /* Disable TIM4 update interrupt during the reset to prevent the
     * overflow ISR from firing on the CNT=0 write and miscounting. */
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);

    ENC_RESET_ALL();
    s_right_overflow   = 0;
    s_right_last_cnt   = 0U;
    s_left_prev        = 0;
    s_right_prev       = 0;
    s_left_delta_last  = 0;
    s_right_delta_last = 0;

    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

/* =========================================================================
 * PUBLIC API — CUMULATIVE POSITION
 * ======================================================================= */

/**
 * @brief  Return current signed cumulative count for the left wheel.
 *
 * @details TIM2 is 32-bit — direct read, no overflow handling needed.
 *          Cast to int32_t to get two's-complement signed value.
 *          Applies LEFT_ENC_POL.
 */
int32_t enc_left_count(void)
{
    int32_t raw = (int32_t)ENC_L_CNT;    /* TIM2->CNT, 32-bit direct read */
    return raw * LEFT_ENC_POL;
}

/**
 * @brief  Return current signed cumulative count for the right wheel.
 *
 * @details Uses read_right_count() for race-safe 16-bit + overflow assembly.
 */
int32_t enc_right_count(void)
{
    return read_right_count();
}

/* =========================================================================
 * PUBLIC API — DELTA  (call once per 1 ms control tick)
 * ======================================================================= */

/**
 * @brief  Return left wheel counts since the previous call.
 *
 * @details Caches the current count, computes delta, stores for speed
 *          estimation.  Call exactly once per CTRL_LOOP_HZ tick.
 */
int32_t enc_left_delta(void)
{
    int32_t now          = enc_left_count();
    int32_t delta        = now - s_left_prev;
    s_left_prev          = now;
    s_left_delta_last    = delta;
    return delta;
}

/**
 * @brief  Return right wheel counts since the previous call.
 */
int32_t enc_right_delta(void)
{
    int32_t now          = read_right_count();
    int32_t delta        = now - s_right_prev;
    s_right_prev         = now;
    s_right_delta_last   = delta;
    return delta;
}

/* =========================================================================
 * PUBLIC API — DISTANCE (mm)
 * ======================================================================= */

/**
 * @brief  Return cumulative left wheel travel in mm since last reset.
 */
float enc_left_mm(void)
{
    return (float)enc_left_count() * MM_PER_COUNT;
}

/**
 * @brief  Return cumulative right wheel travel in mm since last reset.
 */
float enc_right_mm(void)
{
    return (float)enc_right_count() * MM_PER_COUNT;
}

/**
 * @brief  Return average of both wheel distances in mm since last reset.
 */
float enc_avg_mm(void)
{
    return (enc_left_mm() + enc_right_mm()) * 0.5f;
}

/**
 * @brief  Return signed left-minus-right tracking error in mm.
 *
 * @details Positive → left wheel ahead → robot drifting right.
 *          Negative → right wheel ahead → robot drifting left.
 *          Used directly as the error signal for the straightness PID
 *          in motion.c.
 */
float enc_tracking_error_mm(void)
{
    return enc_left_mm() - enc_right_mm();
}

/* =========================================================================
 * PUBLIC API — SPEED ESTIMATION
 * ======================================================================= */

/**
 * @brief  Estimate left wheel speed in mm/s.
 *
 * @details Uses the last delta cached by enc_left_delta().
 *          Only accurate when enc_left_delta() was called at exactly
 *          CTRL_LOOP_HZ (1 kHz):
 *            speed = delta_counts × MM_PER_COUNT × CTRL_LOOP_HZ
 *          Returns 0.0f before first enc_left_delta() call.
 */
float enc_left_speed_mmps(void)
{
    if (!s_initialised) { return 0.0f; }
    return (float)s_left_delta_last * MM_PER_COUNT * (float)CTRL_LOOP_HZ;
}

/**
 * @brief  Estimate right wheel speed in mm/s.
 *
 * @details Uses the last delta cached by enc_right_delta().
 */
float enc_right_speed_mmps(void)
{
    if (!s_initialised) { return 0.0f; }
    return (float)s_right_delta_last * MM_PER_COUNT * (float)CTRL_LOOP_HZ;
}

/* =========================================================================
 * PUBLIC API — ISR HOOK
 * ======================================================================= */

/**
 * @brief  TIM4 overflow/underflow ISR hook.
 *
 * @details Called from HAL_TIM_PeriodElapsedCallback in main.c when
 *          htim->Instance == TIM4.  Runs inside a hardware interrupt.
 *
 *          DIRECTION DETERMINATION
 *          ────────────────────────
 *          After a wrap the new TIM4->CNT value tells us which way:
 *
 *          Forward overflow  (65535 → 0):
 *            Before: CNT was near 65535.
 *            After:  CNT is near 0 (0x0000 to 0x3FFF range).
 *            Action: s_right_overflow++
 *
 *          Reverse underflow (0 → 65535):
 *            Before: CNT was near 0.
 *            After:  CNT is near 65535 (0xC000 to 0xFFFF range).
 *            Action: s_right_overflow--
 *
 *          The threshold 0x4000 (16384) / 0xC000 (49152) gives a dead
 *          zone in the centre of the count range where a normal update
 *          could not be confused with a wrap event.
 */
void encoders_tim4_ovf_irq(void)
{
    uint16_t cnt = ENC_R_CNT;   /* Read CNT immediately — it has just wrapped */

    if (cnt < 0x4000U)
    {
        /* Counter crossed 65535 → 0: forward overflow */
        s_right_overflow++;
    }
    else if (cnt > 0xC000U)
    {
        /* Counter crossed 0 → 65535: reverse underflow */
        s_right_overflow--;
    }
    /* Values between 0x4000 and 0xC000 should not occur on a wrap event.
     * If they do, ignore — a spurious IRQ from noise or startup.         */

    s_right_last_cnt = cnt;
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate an EncoderState_t snapshot of current encoder state.
 *
 * @param[out] state  Pointer to struct to fill. Must not be NULL.
 */
void encoders_get_state(EncoderState_t *state)
{
    if (state == NULL) { return; }

    state->count_left    = enc_left_count();
    state->count_right   = enc_right_count();
    state->delta_left    = s_left_delta_last;
    state->delta_right   = s_right_delta_last;
    state->dist_left_mm  = enc_left_mm();
    state->dist_right_mm = enc_right_mm();
    state->dist_avg_mm   = enc_avg_mm();
    state->overflow_count = (uint32_t)((s_right_overflow < 0)
                             ? -s_right_overflow
                             :  s_right_overflow);
}
