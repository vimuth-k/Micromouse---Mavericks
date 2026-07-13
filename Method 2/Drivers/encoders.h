/**
 * @file    encoders.h
 * @brief   Quadrature encoder reading — public API.
 *
 * @details Provides signed 32-bit position counts and derived distance
 *          measurements for both drive wheels.
 *
 *          HARDWARE
 *          ────────
 *          Left  wheel : TIM2 (32-bit, PA0/PA1) — no overflow handling needed.
 *          Right wheel : TIM4 (16-bit, PB6/PB7) — overflow tracked via IRQ.
 *
 *          Both timers run in encoder mode TI1+TI2 (×4 quadrature):
 *          every rising and falling edge of both channels is counted,
 *          giving 4 × 7 PPR × 30:1 gear = 840 counts per wheel revolution.
 *
 *          SIGN CONVENTION
 *          ───────────────
 *          Positive count = wheel moving in the robot-forward direction,
 *          after applying LEFT_ENC_POL / RIGHT_ENC_POL from config.h.
 *          Delta is positive when the robot is moving forward.
 *
 *          USAGE IN THE CONTROL LOOP
 *          ──────────────────────────
 *          The 1 kHz tick in motion.c calls enc_left_delta() and
 *          enc_right_delta() exactly once per tick to get the number
 *          of counts travelled in the last millisecond.  These deltas
 *          feed directly into the speed and straightness PID loops.
 *          Do not call enc_left_count() / enc_right_count() inside the
 *          ISR unless you need cumulative position — use the delta API.
 *
 *          THREAD SAFETY
 *          ─────────────
 *          enc_left_count()  is safe to call from both main loop and ISR
 *          because TIM2->CNT is a single 32-bit atomic read on Cortex-M4.
 *
 *          enc_right_count() is NOT atomic — it combines the 16-bit
 *          TIM4->CNT with a volatile overflow counter that is updated
 *          in the TIM4 IRQ.  Read it with interrupts disabled if you
 *          need a consistent snapshot from the main loop.
 *
 *          enc_left_delta() / enc_right_delta() store previous counts
 *          in static variables.  Call each exactly once per 1 ms tick.
 *          Calling more than once per tick returns a smaller-than-actual
 *          delta.  Calling from multiple contexts causes data races.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          pins.h   — ENC_L_CNT, ENC_R_CNT, ENC_RESET_ALL()
 *          config.h — MM_PER_COUNT, LEFT_ENC_POL, RIGHT_ENC_POL,
 *                     COUNTS_PER_REV, COUNTS_PER_CELL
 *          main.h   — htim2, htim4 externs
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  Full encoder state snapshot — populated by encoders_get_state().
 */
typedef struct
{
    int32_t  count_left;       /**< Cumulative signed count, left  wheel  */
    int32_t  count_right;      /**< Cumulative signed count, right wheel  */
    int32_t  delta_left;       /**< Counts since last snapshot call       */
    int32_t  delta_right;      /**< Counts since last snapshot call       */
    float    dist_left_mm;     /**< Cumulative distance, left  wheel (mm) */
    float    dist_right_mm;    /**< Cumulative distance, right wheel (mm) */
    float    dist_avg_mm;      /**< Average of both wheels (mm)           */
    uint32_t overflow_count;   /**< TIM4 16-bit overflow event counter    */
} EncoderState_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Start both encoder timers and zero all state.
 *
 * @details Calls HAL_TIM_Encoder_Start() on both htim2 and htim4.
 *          Resets all delta-tracking static variables.
 *          Must be called once during module bring-up, after TIM2 and
 *          TIM4 have been initialised in main.c.
 *
 * @return MM_OK always.
 */
MmResult_t encoders_init(void);

/**
 * @brief  Zero both hardware counters and all accumulated state.
 *
 * @details Writes 0 to TIM2->CNT and TIM4->CNT, clears the overflow
 *          counter, and resets the delta-tracking previous-count variables.
 *          Call this immediately before every move command so that
 *          enc_avg_mm() returns distance travelled since the move started.
 *
 * @note   Not interrupt-safe: call from the main loop only, with the
 *         control loop timer stopped or when motion is known to be idle.
 */
void encoders_reset(void);

/* =========================================================================
 * POSITION — CUMULATIVE
 * ======================================================================= */

/**
 * @brief  Return the current signed cumulative count for the left wheel.
 *
 * @details Reads TIM2->CNT directly (32-bit atomic read on Cortex-M4)
 *          and applies LEFT_ENC_POL.  Positive = robot has moved forward.
 *          Range: ±2 147 483 647 counts ≈ ±257 km — never wraps in practice.
 *
 * @return int32_t  Signed cumulative count since last encoders_reset().
 */
int32_t enc_left_count(void);

/**
 * @brief  Return the current signed cumulative count for the right wheel.
 *
 * @details Combines TIM4->CNT (16-bit) with an overflow counter that is
 *          incremented/decremented in encoders_tim4_ovf_irq() to provide
 *          a 32-bit range.  Applies RIGHT_ENC_POL.
 *
 * @warning Not atomic — the 16-bit CNT read and the overflow counter read
 *          can be split by the TIM4 IRQ.  This is handled internally by
 *          reading CNT twice and comparing, but for absolute accuracy in
 *          non-ISR contexts disable interrupts around this call.
 *
 * @return int32_t  Signed cumulative count since last encoders_reset().
 */
int32_t enc_right_count(void);

/* =========================================================================
 * POSITION — DELTA  (call once per 1 ms control tick)
 * ======================================================================= */

/**
 * @brief  Return left wheel counts since the previous call.
 *
 * @details Subtracts the internally cached previous count from the
 *          current enc_left_count() and updates the cache.
 *          At 1 kHz and SPD_SEARCH (200 mm/s):
 *            Δcounts = 200 mm/s × 0.001 s / 0.12716 mm = ~1.57 counts/tick
 *          At SPD_RUN3 (750 mm/s):
 *            Δcounts = 750 × 0.001 / 0.12716 ≈ 5.9 counts/tick
 *
 * @return int32_t  Signed delta count. Positive = forward motion.
 */
int32_t enc_left_delta(void);

/**
 * @brief  Return right wheel counts since the previous call.
 *
 * @details Identical to enc_left_delta() but for the right wheel.
 *
 * @return int32_t  Signed delta count. Positive = forward motion.
 */
int32_t enc_right_delta(void);

/* =========================================================================
 * POSITION — DISTANCE (mm)
 * ======================================================================= */

/**
 * @brief  Return cumulative left wheel travel in mm since last reset.
 *
 * @details Converts enc_left_count() to millimetres using MM_PER_COUNT
 *          from config.h.
 *          MM_PER_COUNT = π × 34 / 840 = 0.12716 mm/count.
 *
 * @return float  Distance in mm. Positive = forward.
 */
float enc_left_mm(void);

/**
 * @brief  Return cumulative right wheel travel in mm since last reset.
 *
 * @return float  Distance in mm. Positive = forward.
 */
float enc_right_mm(void);

/**
 * @brief  Return average of left and right wheel distances (mm).
 *
 * @details (enc_left_mm() + enc_right_mm()) / 2.
 *          Represents the distance travelled by the robot's centre point.
 *          Used by the trajectory profiler to determine when to start
 *          braking and when the cell move is complete.
 *
 * @return float  Average distance in mm since last encoders_reset().
 */
float enc_avg_mm(void);

/**
 * @brief  Return signed difference between left and right wheel distances.
 *
 * @details enc_left_mm() − enc_right_mm().
 *          Positive = left wheel has travelled further (robot turning right).
 *          Negative = right wheel has travelled further (robot turning left).
 *          Used directly as the error signal for the straightness PID.
 *
 * @return float  Tracking error in mm. Zero = perfectly straight.
 */
float enc_tracking_error_mm(void);

/* =========================================================================
 * SPEED ESTIMATION (mm/s)
 * ======================================================================= */

/**
 * @brief  Estimate left wheel speed in mm/s from the most recent delta.
 *
 * @details Converts the last delta count to mm/s:
 *            speed_mmps = delta_counts × MM_PER_COUNT × CTRL_LOOP_HZ
 *          Valid only when called at exactly CTRL_LOOP_HZ (1 kHz).
 *          Returns 0.0f if encoders are not yet initialised.
 *
 * @return float  Speed in mm/s. Positive = forward.
 */
float enc_left_speed_mmps(void);

/**
 * @brief  Estimate right wheel speed in mm/s from the most recent delta.
 *
 * @return float  Speed in mm/s. Positive = forward.
 */
float enc_right_speed_mmps(void);

/* =========================================================================
 * INTERRUPT SERVICE ROUTINE HOOK
 * ======================================================================= */

/**
 * @brief  TIM4 overflow/underflow ISR hook — call from stm32f4xx_it.c.
 *
 * @details TIM4 is a 16-bit counter that overflows at 65535 and underflows
 *          at 0.  This function detects which direction the wrap occurred
 *          and adjusts a 32-bit overflow counter accordingly, extending
 *          the effective range of the right encoder to 32 bits.
 *
 *          MUST be called from TIM4_IRQHandler via
 *          HAL_TIM_PeriodElapsedCallback when htim->Instance == TIM4.
 *          Must execute in under 1 µs — it is in the hot ISR path.
 *
 *          How overflow direction is determined:
 *            - If TIM4->CNT is near 0     after the event → forward overflow
 *              (count crossed 65535 → 0, wheel moving forward)
 *            - If TIM4->CNT is near 65535 after the event → reverse underflow
 *              (count crossed 0 → 65535, wheel moving backward)
 */
void encoders_tim4_ovf_irq(void);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate an EncoderState_t snapshot of current encoder state.
 *
 * @details Captures counts, deltas, distances, and overflow counter
 *          in one call for UART logging and OLED display.
 *          Do not call from the 1 kHz ISR — use the individual
 *          accessor functions inside the ISR instead.
 *
 * @param[out] state  Pointer to the struct to fill. Must not be NULL.
 */
void encoders_get_state(EncoderState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ENCODERS_H */
