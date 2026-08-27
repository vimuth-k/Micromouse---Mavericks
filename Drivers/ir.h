/**
 * @file    ir.h
 * @brief   6-channel IR proximity sensor system — public API.
 *
 * @details Manages the full IR sensing pipeline for the micromouse:
 *
 *          HARDWARE
 *          ────────
 *          6 × SFH4545 IR emitters (5° half-angle, 950 nm).
 *          6 × TEFT4300 phototransistors (receivers).
 *          6 × AO3400A N-MOSFETs — one per emitter pair, gate driven
 *              from an independent GPIO so pairs fire independently.
 *
 *          Sensor layout (looking down at robot from above):
 *
 *                          ┌──────────────┐
 *                          │  FRONT       │
 *               L_ANG  ╱  │  RF    LF  │  ╲  R_ANG
 *                      ╲  │            │  ╱
 *               L_SIDE ── │            │ ── R_SIDE
 *                          │            │
 *                          └──────────────┘
 *
 *          ADC ARCHITECTURE
 *          ─────────────────
 *          ADC1 in 7-channel DMA circular scan (ranks match IR_IDX_* in pins.h):
 *            Rank 1  PA2  ADC1_IN2  Right Side  (IR_IDX_RS)
 *            Rank 2  PA3  ADC1_IN3  Left  Side  (IR_IDX_LS)
 *            Rank 3  PA4  ADC1_IN4  Right Front (IR_IDX_RF)
 *            Rank 4  PA5  ADC1_IN5  Left  Front (IR_IDX_LF)
 *            Rank 5  PA6  ADC1_IN6  Right Angle (IR_IDX_R_ANG)
 *            Rank 6  PA7  ADC1_IN7  Left  Angle (IR_IDX_L_ANG)
 *            Rank 7  PB0  ADC1_IN8  Battery ADC (IR_IDX_BATT)
 *          The battery rank is included here so the single DMA scan
 *          captures all analog values simultaneously.  battery.c reads
 *          from the shared buffer via ir_get_raw_batt().
 *
 *          MEASUREMENT METHOD — DIFFERENTIAL (LIT − AMBIENT)
 *          ───────────────────────────────────────────────────
 *          Each emitter pair is fired independently.  For each pair:
 *            1. Read ADC with emitter OFF → ambient value.
 *            2. Wait IR_EMITTER_SETTLE_US (60 µs) for MOSFET/LED rise.
 *            3. Fire that emitter ON.
 *            4. Wait IR_EMITTER_SETTLE_US.
 *            5. Read ADC with emitter ON  → lit value.
 *            6. Turn emitter OFF.
 *            7. differential = lit − ambient  (clamped to 0 if negative).
 *          This cancels ambient light (sunlight, fluorescent, LED panels)
 *          which is the largest source of false readings in the
 *          MicroMaze 3 competition hall.
 *
 *          FIRING ORDER IN ir_update()
 *          ────────────────────────────
 *          All 6 pairs are read in sequence during each call:
 *            RS → LS → RF → LF → R_ANG → L_ANG
 *          Total time per ir_update() call:
 *            6 pairs × 2 settle waits × 60 µs = 720 µs worst case.
 *          This fits inside the 1 ms control loop tick when called once
 *          from motion_1khz_tick().
 *
 *          CALIBRATION DATA
 *          ─────────────────
 *          Two calibration steps are required before the first run:
 *            Step 1 (ir_cal_ambient): robot in open space, no walls.
 *                   Records the ambient ADC value per sensor pair.
 *            Step 2 (ir_cal_wall):   robot centred in start cell.
 *                   Records differential values at 90 mm from each wall.
 *                   Computes detection thresholds = 40 % of wall values.
 *          Calibration data is persisted to Flash sector 7 via
 *          ir_cal_save() and loaded on boot via ir_cal_load().
 *
 *          WALL DETECTION API
 *          ───────────────────
 *          High-level boolean functions for the navigation layer:
 *            ir_wall_front()       true when front pair exceeds threshold
 *            ir_wall_left()        true when left side pair exceeds threshold
 *            ir_wall_right()       true when right side pair exceeds threshold
 *            ir_wall_front_close() true when front wall within ~90 mm
 *          Error signal functions for the PID layer:
 *            ir_front_error()      FL − FR: nose angle to front wall
 *            ir_side_error()       LS − RS: lateral offset between walls
 *
 *          DEPENDENCIES
 *          ─────────────
 *          pins.h   — IR_EMIT_xx_ON/OFF, IR_IDX_*, IR_ADC_BUF_LEN,
 *                     IR_ALL_EMITTERS_OFF, IR_ADC_INSTANCE
 *          config.h — IR_THRESH_*, IR_EMITTER_SETTLE_US, IR_NOISE_FLOOR,
 *                     FLASH_CAL_ADDR, FLASH_CAL_MAGIC, FLASH_CAL_SECTOR
 *          utils.h  — delay_us()
 *          main.h   — hadc1 extern
 *
 * @note    File is named ir.h / ir.c to match the project structure
 *          (Drivers/ir.h) as defined in the module plan.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef IR_H
#define IR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

/* =========================================================================
 * CONSTANTS
 * ======================================================================= */

/** Number of IR sensor pairs (SFH4545 + TEFT4300). */
#define IR_NUM_PAIRS            6U

/* Sensor pair indices — use these instead of raw IR_IDX_* from pins.h
 * to keep navigation code readable. */
#define IR_RS      0U   /**< Right Side  — side wall detection, right     */
#define IR_LS      1U   /**< Left  Side  — side wall detection, left      */
#define IR_RF      2U   /**< Right Front — front wall detect + alignment  */
#define IR_LF      3U   /**< Left  Front — front wall detect + alignment  */
#define IR_R_ANG   4U   /**< Right Angle — diagonal wall detection        */
#define IR_L_ANG   5U   /**< Left  Angle — diagonal wall detection        */

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  Calibration data stored in Flash sector 7.
 *
 * @details The struct is written word-by-word to FLASH_CAL_ADDR.
 *          The magic field is checked on load — if it does not match
 *          FLASH_CAL_MAGIC, the data is invalid and defaults are used.
 *          Size must be a multiple of 4 bytes for word-aligned writes.
 */
typedef struct
{
    uint32_t magic;                     /**< FLASH_CAL_MAGIC (0xCAFEBEEF) */
    uint16_t ambient[IR_NUM_PAIRS];     /**< ADC value, emitter OFF, open space */
    uint16_t wall_90mm[IR_NUM_PAIRS];   /**< Differential at 90 mm from wall */
    uint16_t threshold[IR_NUM_PAIRS];   /**< Detect threshold (40 % of wall_90mm) */
    uint16_t threshold_close;           /**< Front-close threshold          */
    uint16_t padding;                   /**< Pad to 4-byte boundary         */
} IrCalData_t;

/**
 * @brief  Complete sensor snapshot — populated by ir_get_snapshot().
 */
typedef struct
{
    uint16_t ambient[IR_NUM_PAIRS];     /**< Last ambient readings          */
    uint16_t lit[IR_NUM_PAIRS];         /**< Last lit readings              */
    uint16_t diff[IR_NUM_PAIRS];        /**< Last differential values       */
    bool     wall_left;
    bool     wall_right;
    bool     wall_front;
    bool     wall_front_close;
    float    front_error;               /**< LF − RF imbalance              */
    float    side_error;                /**< LS − RS imbalance              */
} IrSnapshot_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the IR sensor module.
 *
 * @details Turns all emitters OFF, starts ADC1 DMA circular scan,
 *          attempts to load calibration data from Flash.
 *          If no valid calibration is found, installs conservative
 *          defaults from config.h and logs a warning.
 *          Call once during system bring-up after hadc1 is initialised.
 *
 * @return MM_OK          ADC DMA started, calibration loaded or defaults applied.
 * @return MM_ERR_DRIVER  ADC DMA failed to start (hardware fault).
 */
MmResult_t ir_init(void);

/**
 * @brief  DMA transfer-complete callback — call from HAL_ADC_ConvCpltCallback.
 *
 * @details Sets an internal flag indicating the DMA buffer holds a
 *          fresh complete scan.  The next ir_update() call will see
 *          the updated values.  Executes in ISR context — must be fast.
 */
void ir_dma_complete_callback(void);

/* =========================================================================
 * MEASUREMENT
 * ======================================================================= */

/**
 * @brief  Fire all six sensor pairs and update differential readings.
 *
 * @details Called from motion_1khz_tick() in motion.c — runs inside
 *          the 1 kHz TIM5 interrupt.  Executes in approximately 720 µs.
 *
 *          For each of the 6 pairs in order (RS, LS, RF, LF, R_ANG, L_ANG):
 *            1. Read ADC DMA buffer → ambient.
 *            2. Fire that pair's emitter ON.
 *            3. Wait IR_EMITTER_SETTLE_US.
 *            4. Read ADC DMA buffer → lit.
 *            5. Turn emitter OFF.
 *            6. diff = clamp(lit − ambient, 0, 4095).
 *
 *          Results are stored in internal buffers and available
 *          immediately via ir_get_diff() and the wall detection API.
 */
void ir_update(void);

/**
 * @brief  Return the most recent differential ADC value for one pair.
 *
 * @details Differential = lit − ambient, clamped to [0, 4095].
 *          Higher value = more IR light reflected = wall is closer.
 *          Zero = no wall or below noise floor.
 *
 * @param  idx  Sensor index: IR_RS, IR_LS, IR_RF, IR_LF, IR_R_ANG, IR_L_ANG.
 * @return uint16_t  12-bit differential ADC count. 0 if idx is out of range.
 */
uint16_t ir_get_diff(uint8_t idx);

/**
 * @brief  Return the raw battery ADC value from the shared DMA buffer.
 *
 * @details Slot IR_IDX_BATT (rank 7) in the ADC scan.
 *          battery.c calls this to get the latest voltage reading without
 *          needing its own ADC conversion.
 *
 * @return uint16_t  12-bit raw ADC count for the battery voltage divider.
 */
uint16_t ir_get_raw_batt(void);

/* =========================================================================
 * WALL DETECTION — BOOLEAN
 * ======================================================================= */

/**
 * @brief  True when a wall is present directly in front of the robot.
 *
 * @details Returns true when IR_RF diff OR IR_LF diff exceeds the
 *          calibrated front threshold.  Using OR (not AND) ensures a
 *          wall is detected even when the robot is slightly off-centre.
 *
 * @return bool  true = front wall present.
 */
bool ir_wall_front(void);

/**
 * @brief  True when a wall is present on the left side.
 *
 * @details Returns true when IR_LS diff exceeds the calibrated side threshold.
 *
 * @return bool  true = left wall present.
 */
bool ir_wall_left(void);

/**
 * @brief  True when a wall is present on the right side.
 *
 * @details Returns true when IR_RS diff exceeds the calibrated side threshold.
 *
 * @return bool  true = right wall present.
 */
bool ir_wall_right(void);

/**
 * @brief  True when the front wall is within approximately 90 mm.
 *
 * @details Returns true when BOTH IR_RF AND IR_LF exceed the
 *          close-range threshold (IR_THRESH_FRONT_CLOSE from config.h,
 *          or the calibrated override).  Using AND here requires both
 *          front sensors to agree — reduces false triggers from sharp
 *          corners when the robot is still approaching at an angle.
 *          Used by turn.c to trigger front-wall alignment before turns.
 *
 * @return bool  true = front wall within ~90 mm.
 */
bool ir_wall_front_close(void);

/* =========================================================================
 * ERROR SIGNALS — FOR PID CONTROLLERS
 * ======================================================================= */

/**
 * @brief  Front sensor balance error for heading alignment.
 *
 * @details Returns: (float)diff[IR_LF] − (float)diff[IR_RF]
 *
 *          Interpretation:
 *            Positive → LF reading higher → robot nose angled RIGHT
 *                       (left sensor closer to wall → steer left)
 *            Negative → RF reading higher → robot nose angled LEFT
 *            Zero     → robot square to front wall
 *
 *          Used by turn.c in align_to_front_wall() to square up before
 *          turns.  Also used by motion.c as a heading supplement when
 *          front wall is present.
 *
 * @return float  Signed balance error in ADC counts. Range ~[-4095, +4095].
 */
float ir_front_error(void);

/**
 * @brief  Side sensor balance error for lateral wall-centering.
 *
 * @details Returns: (float)diff[IR_LS] − (float)diff[IR_RS]
 *
 *          Interpretation:
 *            Positive → LS reading higher → robot closer to LEFT wall
 *                       → steer right to re-centre
 *            Negative → RS reading higher → robot closer to RIGHT wall
 *                       → steer left to re-centre
 *            Zero     → robot centred between walls
 *
 *          Used by motion.c when both ir_wall_left() AND ir_wall_right()
 *          are true.  When only one side wall is present, wall-centering
 *          is disabled and the straightness PID takes over.
 *
 * @return float  Signed balance error in ADC counts. Range ~[-4095, +4095].
 */
float ir_side_error(void);

/* =========================================================================
 * CALIBRATION
 * ======================================================================= */

/**
 * @brief  Capture ambient (emitter-off) baseline for all six pairs.
 *
 * @details Robot must be placed in OPEN SPACE with no walls within 40 cm.
 *          Fires NO emitters.  Reads the DMA buffer directly.
 *          Stores raw ADC values in the calibration struct ambient[] array.
 *          Call ir_cal_save() afterward to persist to Flash.
 *
 * @note   Called from calibration.c Mode 1, Step 1.
 */
void ir_cal_ambient(void);

/**
 * @brief  Capture wall-reference readings at 90 mm distance.
 *
 * @details Robot must be placed in the START CELL (three walls present:
 *          left, right, and front, all at 90 mm from sensor face).
 *          Runs a full ir_update() to fire all pairs.
 *          Stores differential readings in wall_90mm[].
 *          Computes thresholds = 40 % of wall_90mm[] (floor: 60 counts).
 *          Computes threshold_close = 80 % of the average of RF + LF.
 *          Call ir_cal_save() afterward to persist to Flash.
 *
 * @note   Called from calibration.c Mode 1, Step 2.
 */
void ir_cal_wall(void);

/**
 * @brief  Write calibration data to Flash sector 7.
 *
 * @details Erases FLASH_CAL_SECTOR, then writes the IrCalData_t struct
 *          word-by-word starting at FLASH_CAL_ADDR.
 *          Sets magic = FLASH_CAL_MAGIC before writing.
 *          Blocks until erase and write complete (~100 ms on F411).
 *
 * @return MM_OK           Data written successfully.
 * @return MM_ERR_STORAGE  Flash erase or write failed.
 */
MmResult_t ir_cal_save(void);

/**
 * @brief  Load calibration data from Flash sector 7.
 *
 * @details Reads the IrCalData_t struct at FLASH_CAL_ADDR.
 *          Validates the magic field.
 *          If valid, copies data into the active calibration struct.
 *          If invalid, returns MM_ERR_NOT_FOUND — caller should fall
 *          back to config.h defaults and warn the user.
 *
 * @return MM_OK            Valid calibration loaded.
 * @return MM_ERR_NOT_FOUND Magic invalid — no calibration in Flash.
 */
MmResult_t ir_cal_load(void);

/**
 * @brief  Return a pointer to the active calibration data (read-only).
 *
 * @details Used by diagnostics.c to display calibration values on the
 *          OLED and via UART without copying the struct.
 *
 * @return const IrCalData_t*  Pointer to internal calibration struct.
 */
const IrCalData_t *ir_cal_get(void);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a complete sensor snapshot for logging and display.
 *
 * @details Captures all ambient, lit, and differential values plus the
 *          boolean wall flags and error signals in one atomic-enough call.
 *          Do not call from inside the 1 kHz ISR.
 *
 * @param[out] snap  Pointer to the IrSnapshot_t struct to fill. Not NULL.
 */
void ir_get_snapshot(IrSnapshot_t *snap);

#ifdef __cplusplus
}
#endif

#endif /* IR_H */
