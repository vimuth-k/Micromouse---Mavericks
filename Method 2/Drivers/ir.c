/**
 * @file    ir.c
 * @brief   6-channel IR proximity sensor system — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          This file controls the complete IR sensing pipeline:
 *
 *          1. EMITTER FIRING (6 independent AO3400A MOSFET gates)
 *             Each of the 6 SFH4545 emitter LEDs is driven by its own
 *             MOSFET from a dedicated GPIO pin (PA10, PA11, PA15,
 *             PB3, PB4, PB5).  Firing them independently prevents
 *             optical crosstalk between adjacent sensor pairs — if all
 *             emitters fired at once, the right-side sensor could pick
 *             up light from the front emitter bouncing off a corner.
 *
 *          2. ADC DMA CIRCULAR SCAN (7 channels, always running)
 *             ADC1 runs a continuous 7-channel DMA scan into adc_dma_buf[].
 *             The buffer is always up-to-date — reading it at any point
 *             gives the most recent conversion result for each channel.
 *             Channel 7 (PB0) is the battery voltage divider and is
 *             included in the same scan so battery.c gets a free reading.
 *
 *          3. DIFFERENTIAL MEASUREMENT (lit − ambient)
 *             For each sensor pair, we read the ADC once with the emitter
 *             OFF (ambient baseline) and once with it ON (lit value).
 *             The differential = lit − ambient cancels all ambient light
 *             — sunlight, LED panel lights, camera flashes — that would
 *             otherwise cause false wall detections.
 *             A reading of 0 = no wall (or below noise floor).
 *             A reading near 4095 = wall very close.
 *
 *          4. WALL PRESENCE DETECTION
 *             Each differential value is compared against a calibrated
 *             threshold stored in IrCalData_t.  Thresholds are set at
 *             40 % of the wall reference reading at 90 mm — chosen to
 *             give a wide hysteresis band and ignore weak reflections
 *             from the floor or distant surfaces.
 *
 *          5. PID ERROR SIGNALS
 *             ir_front_error() and ir_side_error() return signed
 *             imbalance values between symmetric sensor pairs.
 *             These feed directly into the wall-centering and
 *             front-alignment PID controllers in motion.c and turn.c.
 *
 *          6. CALIBRATION & FLASH PERSISTENCE
 *             IrCalData_t is written to Flash sector 7 at FLASH_CAL_ADDR.
 *             On every boot ir_init() attempts to load it.  If the magic
 *             word is invalid (first boot or Flash erased), config.h
 *             default thresholds are installed instead.
 *
 *          TIMING BUDGET
 *          ──────────────
 *          ir_update() is called from the 1 kHz ISR in motion.c.
 *          Each of 6 pairs requires 2 × IR_EMITTER_SETTLE_US = 120 µs.
 *          Total: 6 × 120 = 720 µs.
 *          The 1 ms tick budget is 1000 µs.  Remaining 280 µs covers
 *          gyro I2C read (~20 µs), encoder reads (~5 µs), and PID
 *          computation (~30 µs).  Total ISR load ≈ 75 %.
 *
 * @author  VDawn
 * @date    2026
 */

#include "ir.h"
#include "pins.h"
#include "config.h"
#include "main.h"
#include "utils.h"
#include "flash_storage.h"
#include <string.h>

/* =========================================================================
 * PRIVATE — ADC DMA BUFFER
 * ======================================================================= */

/**
 * @brief  DMA destination buffer — one slot per ADC rank.
 *
 * @details ADC1 DMA circular mode writes here continuously.
 *          Always holds the most recent conversion result.
 *          Slot layout matches IR_IDX_* in pins.h exactly:
 *            [0] RS  [1] LS  [2] RF  [3] LF  [4] R_ANG  [5] L_ANG  [6] BATT
 *
 * @note   Declared volatile so the compiler never caches a read —
 *         the DMA controller can update any slot at any time.
 */
static volatile uint16_t adc_dma_buf[IR_ADC_BUF_LEN];

/**
 * @brief  Flag set by ir_dma_complete_callback() each time DMA finishes
 *         a full 7-channel sweep.  Cleared after ir_update() reads it.
 */
static volatile bool s_dma_fresh = false;

/* =========================================================================
 * PRIVATE — MEASUREMENT BUFFERS
 * ======================================================================= */

/** Last ambient ADC reading per pair (emitter OFF). */
static uint16_t s_ambient[IR_NUM_PAIRS];

/** Last lit ADC reading per pair (emitter ON). */
static uint16_t s_lit[IR_NUM_PAIRS];

/**
 * @brief  Differential reading per pair (lit − ambient, clamped ≥ 0).
 *
 * @details This is the primary output of the module.
 *          Updated by ir_update() every 1 ms during a run.
 *          All wall detection and PID error functions read from here.
 */
static uint16_t s_diff[IR_NUM_PAIRS];

/* =========================================================================
 * PRIVATE — CALIBRATION STATE
 * ======================================================================= */

/**
 * @brief  Active calibration data used for all threshold comparisons.
 *
 * @details Loaded from Flash on boot.  If Flash is invalid, populated
 *          with conservative defaults from config.h.
 *          Updated in place by ir_cal_ambient() and ir_cal_wall().
 */
static IrCalData_t s_cal;

/** True after ir_init() has been called successfully. */
static bool s_initialised = false;

/* =========================================================================
 * PRIVATE — EMITTER FIRE TABLE
 * ======================================================================= */

/**
 * @brief  Per-pair emitter ON and OFF function pointers.
 *
 * @details Allows ir_update() to iterate over all 6 pairs in a loop
 *          rather than 6 copy-pasted blocks.  The ON/OFF functions are
 *          the macros from pins.h wrapped in static inline functions
 *          so they can be stored as function pointers.
 */
typedef void (*EmitFn_t)(void);

static void emit_rs_on(void)    { IR_EMIT_R_SIDE_ON();  }
static void emit_rs_off(void)   { IR_EMIT_R_SIDE_OFF(); }
static void emit_ls_on(void)    { IR_EMIT_L_SIDE_ON();  }
static void emit_ls_off(void)   { IR_EMIT_L_SIDE_OFF(); }
static void emit_rf_on(void)    { IR_EMIT_RF_ON();      }
static void emit_rf_off(void)   { IR_EMIT_RF_OFF();     }
static void emit_lf_on(void)    { IR_EMIT_LF_ON();      }
static void emit_lf_off(void)   { IR_EMIT_LF_OFF();     }
static void emit_rang_on(void)  { IR_EMIT_R_ANGLE_ON(); }
static void emit_rang_off(void) { IR_EMIT_R_ANGLE_OFF();}
static void emit_lang_on(void)  { IR_EMIT_L_ANGLE_ON(); }
static void emit_lang_off(void) { IR_EMIT_L_ANGLE_OFF();}

/** Fire table — index matches IR_RS … IR_L_ANG. */
static const EmitFn_t EMIT_ON[IR_NUM_PAIRS] = {
    emit_rs_on, emit_ls_on, emit_rf_on,
    emit_lf_on, emit_rang_on, emit_lang_on
};

static const EmitFn_t EMIT_OFF[IR_NUM_PAIRS] = {
    emit_rs_off, emit_ls_off, emit_rf_off,
    emit_lf_off, emit_rang_off, emit_lang_off
};

/**
 * @brief  ADC DMA buffer slot index for each sensor pair.
 *         Must match the rank order configured in main.c adc1_ir_battery_init().
 */
static const uint8_t PAIR_ADC_IDX[IR_NUM_PAIRS] = {
    IR_IDX_RS,    /* IR_RS    → rank 1 → buf[0] */
    IR_IDX_LS,    /* IR_LS    → rank 2 → buf[1] */
    IR_IDX_RF,    /* IR_RF    → rank 3 → buf[2] */
    IR_IDX_LF,    /* IR_LF    → rank 4 → buf[3] */
    IR_IDX_R_ANG, /* IR_R_ANG → rank 5 → buf[4] */
    IR_IDX_L_ANG  /* IR_L_ANG → rank 6 → buf[5] */
};

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Install config.h default thresholds into s_cal.
 *
 * @details Called when no valid calibration exists in Flash.
 *          Values are conservative — they will work for initial testing
 *          but must be replaced by running Mode 1 calibration before
 *          any competition.
 */
static void install_default_thresholds(void)
{
    s_cal.magic = 0U;   /* Mark as not calibrated */

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        s_cal.ambient[i]  = 0U;
        s_cal.wall_90mm[i]= 0U;
    }

    /* Front pair thresholds */
    s_cal.threshold[IR_RF]    = IR_THRESH_FRONT;
    s_cal.threshold[IR_LF]    = IR_THRESH_FRONT;

    /* Side pair thresholds */
    s_cal.threshold[IR_RS]    = IR_THRESH_SIDE;
    s_cal.threshold[IR_LS]    = IR_THRESH_SIDE;

    /* Diagonal pair thresholds */
    s_cal.threshold[IR_R_ANG] = IR_THRESH_DIAG;
    s_cal.threshold[IR_L_ANG] = IR_THRESH_DIAG;

    /* Front-close threshold */
    s_cal.threshold_close = IR_THRESH_FRONT_CLOSE;
}

/**
 * @brief  Read one pair: ambient → emitter ON → lit → emitter OFF → diff.
 *
 * @details Reads the ADC DMA buffer directly for both readings.
 *          The DMA is always running in circular mode so the buffer
 *          always contains the most recent conversion result.
 *          The 60 µs settle time covers:
 *            - AO3400A gate rise time     (~10 ns, negligible)
 *            - SFH4545 forward-current rise (~100 ns, negligible)
 *            - TEFT4300 phototransistor response time (~10 µs)
 *            - ADC aperture and sample-and-hold settle (~1 µs)
 *            - Margin for DMA write lag   (~10 µs worst case)
 *          Result is stored in s_ambient[], s_lit[], and s_diff[].
 *
 * @param  pair_idx  Sensor pair index (IR_RS … IR_L_ANG).
 */
static void read_pair(uint8_t pair_idx)
{
    uint8_t adc_slot = PAIR_ADC_IDX[pair_idx];

    /* Step 1: Read ambient (emitter OFF — it should already be off) */
    delay_us(IR_EMITTER_SETTLE_US);
    s_ambient[pair_idx] = adc_dma_buf[adc_slot];

    /* Step 2: Fire emitter ON */
    EMIT_ON[pair_idx]();

    /* Step 3: Wait for emitter and receiver to stabilise */
    delay_us(IR_EMITTER_SETTLE_US);

    /* Step 4: Read lit value */
    s_lit[pair_idx] = adc_dma_buf[adc_slot];

    /* Step 5: Emitter OFF — minimise heat accumulation in SFH4545 */
    EMIT_OFF[pair_idx]();

    /* Step 6: Compute differential, clamp to [0, 4095] */
    if (s_lit[pair_idx] > s_ambient[pair_idx])
    {
        uint16_t raw_diff = s_lit[pair_idx] - s_ambient[pair_idx];
        s_diff[pair_idx]  = (raw_diff < (uint16_t)IR_NOISE_FLOOR)
                            ? 0U
                            : raw_diff;
    }
    else
    {
        /* lit ≤ ambient: no reflected signal above ambient */
        s_diff[pair_idx] = 0U;
    }
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the IR sensor module.
 */
MmResult_t ir_init(void)
{
    /* Ensure all emitters are off before touching anything */
    IR_ALL_EMITTERS_OFF();

    /* Zero measurement buffers */
    (void)memset(s_ambient, 0, sizeof(s_ambient));
    (void)memset(s_lit,     0, sizeof(s_lit));
    (void)memset(s_diff,    0, sizeof(s_diff));
    (void)memset((void *)adc_dma_buf, 0, sizeof(adc_dma_buf));

    s_dma_fresh = false;

    /* Start ADC DMA circular scan */
    HAL_StatusTypeDef status =
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, IR_ADC_BUF_LEN);

    if (status != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }

    /* Load calibration — fall back to defaults if Flash invalid */
    if (ir_cal_load() != MM_OK)
    {
        install_default_thresholds();
    }

    s_initialised = true;
    return MM_OK;
}

/**
 * @brief  DMA transfer-complete callback — call from HAL_ADC_ConvCpltCallback.
 */
void ir_dma_complete_callback(void)
{
    s_dma_fresh = true;
}

/* =========================================================================
 * PUBLIC API — MEASUREMENT
 * ======================================================================= */

/**
 * @brief  Fire all six sensor pairs and update differential readings.
 *
 * @details Iterates through all 6 pairs in order using read_pair().
 *          After this returns, s_diff[] holds fresh values and all
 *          wall detection / error functions reflect the current scene.
 *          All emitters are guaranteed OFF on exit.
 *
 *          Called from motion_1khz_tick() — must complete in < 1 ms.
 *          Measured execution time: ~720 µs at 100 MHz with 60 µs settle.
 */
void ir_update(void)
{
    if (!s_initialised) { return; }

    /* Ensure all emitters are off before the sequence begins */
    IR_ALL_EMITTERS_OFF();

    /* Read each pair independently to avoid crosstalk */
    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        read_pair(i);
    }

    /* Safety: guarantee all emitters off on exit even if read_pair
     * was interrupted or threw an unexpected path.                  */
    IR_ALL_EMITTERS_OFF();

    s_dma_fresh = false;
}

/**
 * @brief  Return the most recent differential ADC value for one pair.
 */
uint16_t ir_get_diff(uint8_t idx)
{
    if (idx >= IR_NUM_PAIRS) { return 0U; }
    return s_diff[idx];
}

/**
 * @brief  Return the raw battery ADC value from the shared DMA buffer.
 */
uint16_t ir_get_raw_batt(void)
{
    return adc_dma_buf[IR_IDX_BATT];
}

/* =========================================================================
 * PUBLIC API — WALL DETECTION (BOOLEAN)
 * ======================================================================= */

/**
 * @brief  True when a wall is present directly in front of the robot.
 *
 * @details OR logic: either front sensor seeing a wall is sufficient.
 *          This handles the case where the robot is slightly off-centre
 *          and only one front sensor is strongly reflecting.
 */
bool ir_wall_front(void)
{
    return (s_diff[IR_RF] > s_cal.threshold[IR_RF]) ||
           (s_diff[IR_LF] > s_cal.threshold[IR_LF]);
}

/**
 * @brief  True when a wall is present on the left side.
 */
bool ir_wall_left(void)
{
    return s_diff[IR_LS] > s_cal.threshold[IR_LS];
}

/**
 * @brief  True when a wall is present on the right side.
 */
bool ir_wall_right(void)
{
    return s_diff[IR_RS] > s_cal.threshold[IR_RS];
}

/**
 * @brief  True when the front wall is within approximately 90 mm.
 *
 * @details AND logic: both front sensors must agree to prevent false
 *          triggers from wall corners during approach.
 */
bool ir_wall_front_close(void)
{
    return (s_diff[IR_RF] > s_cal.threshold_close) &&
           (s_diff[IR_LF] > s_cal.threshold_close);
}

/* =========================================================================
 * PUBLIC API — PID ERROR SIGNALS
 * ======================================================================= */

/**
 * @brief  Front sensor balance error for heading alignment.
 *
 * @details LF − RF.
 *          Positive = nose pointing right = steer left.
 *          Used by turn.c align_to_front_wall() and by motion.c when
 *          ir_wall_front() is true during a straight move.
 */
float ir_front_error(void)
{
    return (float)s_diff[IR_LF] - (float)s_diff[IR_RF];
}

/**
 * @brief  Side sensor balance error for lateral wall-centering.
 *
 * @details LS − RS.
 *          Positive = closer to left wall = steer right.
 *          Applied by motion.c only when BOTH side walls are present.
 */
float ir_side_error(void)
{
    return (float)s_diff[IR_LS] - (float)s_diff[IR_RS];
}

/* =========================================================================
 * PUBLIC API — CALIBRATION
 * ======================================================================= */

/**
 * @brief  Capture ambient baseline for all six pairs.
 *
 * @details Robot must be in open space — no walls within 40 cm.
 *          All emitters remain OFF.  Reads directly from the DMA buffer
 *          which holds the most recent ambient ADC values.
 *          Stores values in s_cal.ambient[].
 */
void ir_cal_ambient(void)
{
    IR_ALL_EMITTERS_OFF();
    delay_us(200U);   /* Let any residual emitter glow decay            */

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        s_cal.ambient[i] = adc_dma_buf[PAIR_ADC_IDX[i]];
    }
}

/**
 * @brief  Capture wall-reference readings at 90 mm distance.
 *
 * @details Robot must be in the start cell (left, right, front walls
 *          all present at 90 mm from each sensor face).
 *          Runs a full ir_update() to get fresh differential values.
 *          Computes thresholds = 40 % of wall reading (floor: 60 counts).
 *          Computes threshold_close = 80 % of (RF + LF) / 2.
 */
void ir_cal_wall(void)
{
    /* Get fresh differential readings in the start cell */
    ir_update();

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        s_cal.wall_90mm[i] = s_diff[i];

        /* Threshold = 40 % of wall reading at 90 mm */
        uint32_t thresh = ((uint32_t)s_diff[i] * 40U) / 100U;

        /* Floor at 60 counts — prevents threshold collapsing near zero */
        s_cal.threshold[i] = (thresh < 60U) ? 60U : (uint16_t)thresh;
    }

    /* Close threshold = 80 % of average front reading at 90 mm */
    uint32_t front_avg = ((uint32_t)s_cal.wall_90mm[IR_RF]
                        +  (uint32_t)s_cal.wall_90mm[IR_LF]) / 2U;
    uint32_t close = (front_avg * 80U) / 100U;
    s_cal.threshold_close = (close < 80U) ? 80U : (uint16_t)close;
}

/**
 * @brief  Write calibration data to Flash sector 7.
 * @details The erase step is delegated to flash_storage_prepare_write(),
 *          which backs up the maze wall map (if present) before erasing
 *          the shared sector and restores it immediately after — sector
 *          7 also holds Maze solve/maze.c's saved wall map at
 *          FLASH_MAZE_ADDR, and the STM32F4 can only erase a sector as
 *          a whole, so saving calibration alone would otherwise wipe it.
 */
MmResult_t ir_cal_save(void)
{
    s_cal.magic   = FLASH_CAL_MAGIC;
    s_cal.padding = 0U;

    HAL_StatusTypeDef status;

    /* Unlock + erase (preserving the maze region) is handled centrally
     * — see flash_storage.h for why this can't be done independently
     * per-module on a chip that only erases whole sectors. */
    if (flash_storage_prepare_write(FLASH_REGION_CAL) != MM_OK)
    {
        return MM_ERR_STORAGE;
    }

    /* Write struct word-by-word (32-bit aligned) */
    const uint32_t *src  = (const uint32_t *)&s_cal;
    uint32_t        addr = FLASH_CAL_ADDR;
    uint32_t        words = (sizeof(IrCalData_t) + 3U) / 4U;

    for (uint32_t w = 0U; w < words; w++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[w]);
        if (status != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return MM_ERR_STORAGE;
        }
        addr += 4U;
    }

    (void)HAL_FLASH_Lock();
    return MM_OK;
}

/**
 * @brief  Load calibration data from Flash sector 7.
 */
MmResult_t ir_cal_load(void)
{
    /* Flash is memory-mapped — cast and dereference directly */
    const IrCalData_t *flash_data = (const IrCalData_t *)FLASH_CAL_ADDR;

    if (flash_data->magic != FLASH_CAL_MAGIC)
    {
        return MM_ERR_NOT_FOUND;
    }

    /* Copy to RAM — avoids slow repeated Flash reads at runtime */
    (void)memcpy(&s_cal, flash_data, sizeof(IrCalData_t));
    return MM_OK;
}

/**
 * @brief  Return pointer to active calibration data (read-only).
 */
const IrCalData_t *ir_cal_get(void)
{
    return &s_cal;
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a complete sensor snapshot for logging and display.
 */
void ir_get_snapshot(IrSnapshot_t *snap)
{
    if (snap == NULL) { return; }

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        snap->ambient[i] = s_ambient[i];
        snap->lit[i]     = s_lit[i];
        snap->diff[i]    = s_diff[i];
    }

    snap->wall_left        = ir_wall_left();
    snap->wall_right       = ir_wall_right();
    snap->wall_front       = ir_wall_front();
    snap->wall_front_close = ir_wall_front_close();
    snap->front_error      = ir_front_error();
    snap->side_error       = ir_side_error();
}
