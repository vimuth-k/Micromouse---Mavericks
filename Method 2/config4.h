/**
 * @file    config.h
 * @brief   MicroMaze 3 — STM32F411CEU6 Black Pill
 *          All tunable robot constants in one place.
 *
 * @details This file is the single source of truth for every numeric
 *          constant in the firmware.  No GPIO definitions appear here —
 *          those live in pins.h.  No HAL calls appear here — this file
 *          is pure C constants and compile-time expressions.
 *
 *          HARDWARE CONFIGURATION
 *          ──────────────────────
 *          MCU     : STM32F411CEU6  (100 MHz, 512 KB Flash, 128 KB RAM)
 *          Motors  : 6 V, 750 RPM output shaft, 30:1 gearbox, 7 PPR encoder
 *          Driver  : TB6612FNG dual H-bridge
 *          IMU     : MPU6500 via I2C1 (400 kHz)
 *          IR emit : 6× SFH4545 (5° half-angle) via AO3400A MOSFETs
 *          IR recv : 6× TEFT4300 phototransistors → ADC1 DMA scan
 *          Display : SSD1306 128×64 OLED via I2C1 (optional)
 *          DIP     : 4-position switch → 16 operating modes
 *          Wheel   : 34 mm diameter rubber
 *
 *          TUNING WORKFLOW
 *          ───────────────
 *          1. Measure WHEEL_SPACING_MM physically with calipers.
 *          2. Run Mode 2 (motor test) — adjust LEFT/RIGHT_MOTOR_POL.
 *          3. Run Mode 3 (straight test) — tune KP/KI/KD_STRAIGHT.
 *          4. Run Mode 4 (turn test)     — tune KP/KI/KD_TURN.
 *          5. Run Mode 1 (IR calibration) — thresholds auto-saved.
 *          6. Run Mode 6 (search)        — tune KP_WALL_CENTER.
 *          7. Run Mode 7-9 (speed runs)  — tune speed profiles.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * SECTION 0 — FIRMWARE VERSION
 * =========================================================================
 * Increment PATCH on bug-fix rebuilds, MINOR on new features,
 * MAJOR on breaking changes to Flash layout or calibration format.
 * ======================================================================= */

#define FW_VERSION_MAJOR        1U
#define FW_VERSION_MINOR        0U
#define FW_VERSION_PATCH        0U

/** Single 32-bit version word: 0x00_MAJOR_MINOR_PATCH */
#define FW_VERSION_WORD \
    ((FW_VERSION_MAJOR << 16U) | (FW_VERSION_MINOR << 8U) | FW_VERSION_PATCH)

/* =========================================================================
 * SECTION 1 — SYSTEM CLOCK
 * =========================================================================
 * STM32F411CEU6: HSE 25 MHz → PLL → 100 MHz SYSCLK.
 * APB1 = 50 MHz (TIM2/TIM4/TIM5 × 2 = 100 MHz timer clock).
 * APB2 = 100 MHz (TIM1 timer clock = 100 MHz).
 * ======================================================================= */

/** Core clock in Hz — must match SystemClock_Config() in main.c */
#define SYSCLK_HZ               100000000UL

/** APB1 timer clock (TIM2, TIM4, TIM5): APB1 × 2 = 100 MHz */
#define APB1_TIM_CLK_HZ         100000000UL

/** APB2 timer clock (TIM1): APB2 × 1 = 100 MHz */
#define APB2_TIM_CLK_HZ         100000000UL

/* =========================================================================
 * SECTION 2 — WHEEL GEOMETRY
 * =========================================================================
 * All distances in millimetres (mm).
 * ======================================================================= */

/**
 * @brief Outer diameter of the rubber tyre in mm.
 * @note  Measure the actual fitted wheel, not the hub.
 *        Typical N20 34 mm rubber wheel has ~0.5 mm tyre compression.
 */
#define WHEEL_DIAMETER_MM       34.0f

/**
 * @brief Distance between the two tyre contact patches on the floor (mm).
 * @warning MEASURE THIS WITH CALIPERS on your actual assembled robot.
 *          This is the most sensitive physical constant.  A 1 mm error
 *          accumulates ~0.7° of heading error per cell at search speed.
 *          Typical value for an 80–100 mm wide chassis: 82.0f–90.0f.
 */
#define WHEEL_SPACING_MM        82.0f

/** Wheel circumference = π × diameter (mm) */
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265f * WHEEL_DIAMETER_MM)

/* =========================================================================
 * SECTION 3 — ENCODER CONSTANTS
 * =========================================================================
 * Motor: 6 V, 750 RPM output shaft, internal 30:1 gearbox.
 * Encoder disk is on the MOTOR shaft (before gearbox): 7 PPR.
 * STM32 timer reads quadrature (×4 edges): effective 28 CPR on motor shaft.
 * After gearbox: 28 × 30 = 840 counts per wheel revolution.
 * ======================================================================= */

/** Encoder pulses per motor shaft revolution (before gearbox) */
#define ENCODER_PPR             7U

/** Gearbox reduction ratio (motor shaft : output shaft) */
#define GEAR_RATIO              30U

/** Quadrature multiplier (STM32 timer counts both edges of both channels) */
#define ENCODER_QUADRATURE      4U

/**
 * @brief Encoder counts per complete wheel revolution.
 * @details CPR = PPR × quadrature × gear_ratio = 7 × 4 × 30 = 840
 */
#define COUNTS_PER_REV          (ENCODER_PPR * ENCODER_QUADRATURE * GEAR_RATIO)

/**
 * @brief Linear distance per encoder count (mm/count).
 * @details mm_per_count = circumference / counts_per_rev
 *          = (π × 34) / 840 = 106.814 / 840 = 0.127160 mm/count
 */
#define MM_PER_COUNT            (WHEEL_CIRCUMFERENCE_MM / (float)COUNTS_PER_REV)

/**
 * @brief Maximum encoder count rate at no-load speed (counts/second).
 * @details 750 RPM output → 750/60 rev/s × 840 counts/rev = 10 500 counts/s
 */
#define MAX_COUNTS_PER_SEC      10500U

/* =========================================================================
 * SECTION 4 — MOTOR AND PWM PARAMETERS
 * =========================================================================
 * TIM1 drives both motor PWM channels (PA8 = left, PA9 = right).
 * Target frequency: 20 kHz (inaudible, efficient for N20 motors).
 * Formula: ARR = (APB2_TIM_CLK / PWM_FREQ) − 1
 *          = (100 000 000 / 20 000) − 1 = 4999
 * ======================================================================= */

/** Motor rated supply voltage (V) */
#define MOTOR_RATED_VOLTAGE_V   6.0f

/** Motor no-load output shaft speed at rated voltage (RPM) */
#define MOTOR_NOLOAD_RPM        750U

/** PWM carrier frequency (Hz) — 20 kHz is above human hearing */
#define PWM_FREQUENCY_HZ        20000U

/** TIM1 auto-reload register value for 20 kHz PWM (prescaler = 0) */
#define PWM_ARR                 4999U

/** Maximum PWM duty count (= ARR = 100 % duty cycle) */
#define PWM_MAX                 4999U

/** Minimum PWM duty count (= 0 % duty cycle, motor coast) */
#define PWM_MIN                 0U

/**
 * @brief Open-loop feed-forward PWM per mm/s (approximate).
 * @details At 100 % PWM the motor reaches ~1335 mm/s (no-load).
 *          Feed-forward: PWM_MAX / MAX_SPEED_MMPS = 4999 / 1335 ≈ 3.74
 *          Used as an initial bias in the speed PID to reduce rise time.
 */
#define PWM_PER_MMPS            3.74f

/**
 * @brief Left motor physical direction correction.
 * @details Set to 1 if left motor spins FORWARD when CCR > 0.
 *          Set to -1 if it spins BACKWARD (motor wired in reverse).
 *          Verified during Mode 2 motor test.
 */
#define LEFT_MOTOR_POL          1

/**
 * @brief Right motor physical direction correction.
 * @details Typically -1 because the right motor is mirrored on the chassis.
 */
#define RIGHT_MOTOR_POL         (-1)

/**
 * @brief Left encoder polarity correction.
 * @details Set to 1 if count INCREASES when left wheel moves forward.
 *          Set to -1 if count DECREASES.
 */
#define LEFT_ENC_POL            1

/**
 * @brief Right encoder polarity correction.
 */
#define RIGHT_ENC_POL           (-1)

/* =========================================================================
 * SECTION 5 — SPEED PROFILES
 * =========================================================================
 * All speeds in mm/s.
 * Theoretical maximum (no-load, 100 % PWM):
 *   750 RPM / 60 × π × 34 mm = 1335 mm/s ≈ 1.34 m/s
 *
 * Working speeds are set well below maximum to allow PID headroom
 * and to leave margin for maze navigation corrections.
 * ======================================================================= */

/** Theoretical no-load maximum wheel speed (mm/s) */
#define SPD_MAX_THEORETICAL     1335.0f

/**
 * @brief Search run speed (mm/s) — slow, maximum reliability.
 * @details Used during Mode 6 exploration to avoid wall hits.
 *          Robot must stop reliably at every cell boundary.
 */
#define SPD_SEARCH              200.0f

/**
 * @brief Return-to-start speed (mm/s) after reaching goal.
 * @details Slightly faster than search — path is already known.
 */
#define SPD_RETURN              250.0f

/**
 * @brief Speed run 1 — first competition speed run (mm/s).
 * @details Conservative first speed run.  Gets a time on the board.
 */
#define SPD_RUN1                400.0f

/**
 * @brief Speed run 2 — second competition speed run (mm/s).
 */
#define SPD_RUN2                600.0f

/**
 * @brief Speed run 3 — maximum competition speed run (mm/s).
 * @details Verify zero wall hits at SPD_RUN2 before enabling this.
 */
#define SPD_RUN3                750.0f

/* =========================================================================
 * SECTION 6 — ACCELERATION PROFILES
 * =========================================================================
 * All values in mm/s².
 * The trapezoidal velocity profile (trajectory.c) ramps speed up at ACCEL
 * and brakes at DECEL.  Decel is higher than accel to ensure the robot
 * always stops within the cell boundary.
 * ======================================================================= */

/** Normal acceleration ramp rate (mm/s²) */
#define ACCEL_NORMAL            3000.0f

/** Normal deceleration ramp rate (mm/s²) */
#define DECEL_NORMAL            4000.0f

/** Search-mode acceleration — gentler for reliability (mm/s²) */
#define ACCEL_SEARCH            2000.0f

/** Search-mode deceleration (mm/s²) */
#define DECEL_SEARCH            3000.0f

/**
 * @brief Minimum speed the velocity profile will ramp down to (mm/s).
 * @details Prevents the robot from stalling at very low PWM.
 *          The motion controller switches to active brake below this.
 */
#define SPD_CREEP               30.0f

/* =========================================================================
 * SECTION 7 — PID GAINS
 * =========================================================================
 * All PID controllers run at CTRL_LOOP_HZ (1 kHz).
 * Gains are therefore per-millisecond — NOT per-second.
 *
 * TUNING ORDER (do not skip):
 *  A. KP_SPEED  → robot reaches target speed without oscillating
 *  B. KI_SPEED  → eliminates steady-state speed error
 *  C. KP_STRAIGHT / KD_STRAIGHT → keeps wheels in sync (straight line)
 *  D. KP_HEADING / KD_HEADING   → gyro drift correction during straights
 *  E. KP_WALL_CENTER            → active centering when both walls present
 *  F. KP_TURN / KD_TURN         → accurate 90° and 180° turns
 * ======================================================================= */

/* ── Speed controller (per wheel, independent L and R instances) ──────── */

/** Proportional gain for wheel speed PID */
#define KP_SPEED                4.5f

/** Integral gain for wheel speed PID */
#define KI_SPEED                9.0f

/** Derivative gain for wheel speed PID */
#define KD_SPEED                0.05f

/** Anti-windup clamp on speed integral accumulator */
#define SPEED_INTEG_LIMIT       2000.0f

/* ── Straightness controller (encoder count difference L − R) ─────────── */

/** Proportional gain — straightness correction */
#define KP_STRAIGHT             2.5f

/** Integral gain — straightness correction (start at 0.0, add slowly) */
#define KI_STRAIGHT             0.0f

/** Derivative gain — straightness correction */
#define KD_STRAIGHT             0.4f

/** Anti-windup clamp on straightness integral */
#define STRAIGHT_INTEG_LIMIT    500.0f

/** Maximum PWM correction the straightness PID can apply (counts) */
#define STRAIGHT_OUTPUT_LIMIT   (PWM_MAX * 0.30f)

/* ── Heading controller (MPU6500 gyro yaw during straights) ───────────── */

/** Proportional gain — heading correction */
#define KP_HEADING              1.8f

/** Integral gain — heading correction */
#define KI_HEADING              0.0f

/** Derivative gain — heading correction */
#define KD_HEADING              0.1f

/** Anti-windup clamp on heading integral (degrees) */
#define HEADING_INTEG_LIMIT     45.0f

/** Maximum PWM correction heading PID can apply (counts) */
#define HEADING_OUTPUT_LIMIT    (PWM_MAX * 0.25f)

/* ── Wall-centering controller (IR side sensors, P-only) ─────────────── */

/**
 * @brief Proportional gain for lateral wall-centering.
 * @details Applied only when both left AND right walls are present.
 *          Error = sensor_val(SENS_L) − sensor_val(SENS_R).
 *          Positive error → robot closer to left wall → steer right.
 */
#define KP_WALL_CENTER          0.35f

/* ── Turn controller (gyro-based pivot turns) ─────────────────────────── */

/** Proportional gain for gyro turn PID */
#define KP_TURN                 12.0f

/** Integral gain for turn PID */
#define KI_TURN                 0.0f

/** Derivative gain for turn PID */
#define KD_TURN                 0.3f

/** Anti-windup clamp on turn integral (degrees) */
#define TURN_INTEG_LIMIT        90.0f

/** Minimum PWM applied during a turn (prevents stall at small errors) */
#define TURN_PWM_MIN            300.0f

/** Maximum PWM applied during a turn */
#define TURN_PWM_MAX            2500.0f

/**
 * @brief Heading error below which the turn is considered complete (degrees).
 * @details Tighter = more accurate turns but longer settle time.
 *          0.8° gives good accuracy without excessive oscillation.
 */
#define TURN_TOLERANCE_DEG      0.8f

/** Time to wait after turn completion before next motion (ms) */
#define TURN_SETTLE_MS          30U

/* ── Front wall alignment controller (IR FL/FR balance) ─────────────── */

/**
 * @brief P-gain for squaring up to the front wall.
 * @details Error = sensor_val(SENS_FL) − sensor_val(SENS_FR).
 *          Applied when wall_front_close() is true before a turn.
 */
#define KP_ALIGN                0.4f

/* =========================================================================
 * SECTION 8 — MAZE CONSTANTS  (MicroMaze 3 specification)
 * =========================================================================
 * Source: MicroMaze 3 Delegate Booklet, IIT RAS 2026.
 * ======================================================================= */

/** Grid size (cells per side) */
#define MAZE_SIZE               16U

/** Physical width of one cell (mm) — 18 cm per MicroMaze 3 spec */
#define CELL_WIDTH_MM           180.0f

/** Half a cell width (mm) — distance from cell centre to wall face */
#define HALF_CELL_MM            90.0f

/** Wall height (mm) */
#define WALL_HEIGHT_MM          50.0f

/** Wall thickness (mm) — 1.2 cm per spec, assume 5 % tolerance */
#define WALL_THICKNESS_MM       12.0f

/**
 * @brief Goal region row bounds (inclusive).
 * @details In a 16×16 maze the goal is the 4-cell area at rows 7–8.
 *          Row 0 = top row (North boundary).
 *          Row 15 = bottom row (South boundary) = start side.
 */
#define GOAL_ROW_MIN            7U
#define GOAL_ROW_MAX            8U

/** Goal region column bounds (inclusive) */
#define GOAL_COL_MIN            7U
#define GOAL_COL_MAX            8U

/** Start cell row (bottom-left corner of the maze) */
#define START_ROW               15U

/** Start cell column */
#define START_COL               0U

/** Start heading: 0=North 1=East 2=South 3=West */
#define START_HEADING           0U

/* =========================================================================
 * SECTION 9 — IR SENSOR CONFIGURATION
 * =========================================================================
 * Hardware: SFH4545 emitters (5° half-angle) + TEFT4300 receivers.
 * Emitters are driven by AO3400A N-MOSFETs from a single GPIO pin.
 * Receivers feed a 7-channel ADC1 DMA scan (6 IR + 1 battery).
 *
 * MEASUREMENT METHOD: differential (lit − ambient) to cancel
 * fluorescent and sunlight interference.
 *
 * THRESHOLDS: filled by calibration routine (Mode 1) and saved to
 * Flash.  The values below are safe fallbacks for first boot only.
 * Replace them after running calibration.
 * ======================================================================= */

/** Number of IR sensor pairs */
#define IR_NUM_SENSORS          6U

/** Emitter ON stabilisation time (µs) — SFH4545 rise time ~100 ns */
#define IR_EMITTER_SETTLE_US    60U

/** Emitter pulse width for one ADC reading (µs) */
#define IR_PULSE_WIDTH_US       100U

/**
 * @defgroup IR_THRESHOLDS Default IR detection thresholds (ADC counts, 12-bit)
 * @details These are CONSERVATIVE defaults for first boot.
 *          Run Mode 1 calibration to generate accurate values.
 *          The calibration routine saves thresholds = 40 % of wall
 *          reference reading at 90 mm distance.
 * @{
 */

/** Front sensor (FL, FR) wall-present threshold */
#define IR_THRESH_FRONT         450U

/** Side sensor (L, R) wall-present threshold */
#define IR_THRESH_SIDE          400U

/** Diagonal sensor (DL, DR) wall-present threshold */
#define IR_THRESH_DIAG          350U

/**
 * @brief Front wall "close" threshold — triggers alignment routine.
 * @details When FL or FR exceeds this, the robot is within ~90 mm
 *          of the front wall.  Use to square up before turns.
 */
#define IR_THRESH_FRONT_CLOSE   700U

/** ADC floor below which a reading is treated as noise (counts) */
#define IR_NOISE_FLOOR          40U

/** @} */

/* =========================================================================
 * SECTION 10 — IMU CONFIGURATION  (MPU6500)
 * =========================================================================
 * Interface: I2C1 at 400 kHz (PB8 SCL, PB9 SDA).
 * AD0 pin tied to GND → I2C address 0x68.
 * ======================================================================= */

/**
 * @brief MPU6500 7-bit I2C address (AD0 = GND).
 * @details Pass (MPU6500_I2C_ADDR << 1) to HAL_I2C_xxx functions.
 */
#define MPU6500_I2C_ADDR        0x68U

/** MPU6500 WHO_AM_I register expected return value */
#define MPU6500_WHO_AM_I_VAL    0x70U

/**
 * @brief Gyroscope full-scale range register value.
 * @details ±500 dps: GYRO_CONFIG register = 0x08.
 *          Sensitivity = 65.5 LSB / (deg/s).
 *          Chosen over ±250 dps because peak turn rate at SPD_RUN3
 *          can exceed 300 deg/s.
 */
#define GYRO_FS_REG_VAL         0x08U

/**
 * @brief Gyroscope sensitivity at ±500 dps full scale.
 * @details LSB per degree per second.  Convert raw reading to deg/s:
 *          angular_rate_dps = (int16_t)raw / GYRO_SENSITIVITY
 */
#define GYRO_SENSITIVITY        65.5f

/**
 * @brief Number of samples averaged during gyro zero-rate calibration.
 * @details Calibration takes GYRO_CAL_SAMPLES × 1 ms = 500 ms.
 *          Robot must remain completely stationary during this time.
 */
#define GYRO_CAL_SAMPLES        500U

/**
 * @brief Control loop time step for gyro integration (seconds).
 * @details Δt = 1 / CTRL_LOOP_HZ = 0.001 s.
 *          Used in mpu6500.c: yaw += (gz_dps − offset) × GYRO_DT
 */
#define GYRO_DT                 0.001f

/**
 * @brief DLPF bandwidth register value (MPU_CONFIG register).
 * @details 0x02 → bandwidth ≈ 92 Hz, delay 3.9 ms.
 *          Reduces motor vibration noise without significant phase lag.
 */
#define GYRO_DLPF_REG_VAL       0x02U

/**
 * @brief Sample rate divider register value (MPU_SMPLRT_DIV).
 * @details With DLPF enabled, internal gyro rate = 1 kHz.
 *          SMPLRT_DIV = 0 → output rate = 1 kHz / (1 + 0) = 1 kHz.
 */
#define GYRO_SMPLRT_DIV         0x00U

/* =========================================================================
 * SECTION 11 — BATTERY MONITORING
 * =========================================================================
 * Battery: 2S Li-ion / LiPo (2 × 3.7 V nominal = 7.4 V).
 * Voltage divider: Vbat ── 10 kΩ ── PB0 (ADC) ── 10 kΩ ── GND
 * ADC reads Vbat / 2.  Reference = 3.3 V, resolution = 12-bit (4095).
 *
 * Conversion: Vbat = raw × (3.3 / 4095) × 2
 *                  = raw × 0.001612903
 * ======================================================================= */

/** 2S cell fully charged voltage (V) */
#define BATT_FULL_V             8.4f

/** 2S cell nominal voltage (V) */
#define BATT_NOMINAL_V          7.4f

/**
 * @brief Low-battery warning threshold (V).
 * @details Buzzer pattern triggers.  Finish current run and land.
 */
#define BATT_LOW_V              7.0f

/**
 * @brief Critical low-battery threshold (V).
 * @details Immediate motor disable to protect cells.
 *          Competition rules allow a battery swap between runs.
 */
#define BATT_CRITICAL_V         6.6f

/**
 * @brief ADC raw-to-volts conversion factor.
 * @details Vbat = (float)adc_raw × BATT_ADC_SCALE
 *          = raw × (3.3 / 4095.0) × 2.0
 *          = raw × 0.001612903f
 */
#define BATT_ADC_SCALE          (3.3f / 4095.0f * 2.0f)

/** Battery ADC update interval (ms) — no need to poll at 1 kHz */
#define BATT_UPDATE_INTERVAL_MS 500U

/* =========================================================================
 * SECTION 12 — FLASH STORAGE
 * =========================================================================
 * STM32F411: 512 KB Flash in 8 sectors (0–7).
 * Sector 7: 0x0806 0000 – 0x0807 FFFF (128 KB).
 * Used for calibration data and optionally maze wall map.
 *
 * Layout within sector 7:
 *   Offset 0x000 : CalibrationData_t  struct
 *   Offset 0x100 : MazeWallMap_t      struct  (optional, large)
 * ======================================================================= */

/** Flash sector number to erase before writing calibration */
#define FLASH_CAL_SECTOR        FLASH_SECTOR_7

/** Base address of calibration storage area */
#define FLASH_CAL_ADDR          0x08060000UL

/** Base address of maze wall map storage (after calibration struct) */
#define FLASH_MAZE_ADDR         0x08060100UL

/**
 * @brief Magic number written as first word of calibration struct.
 * @details If the word at FLASH_CAL_ADDR does not equal this value,
 *          calibration data is considered invalid and defaults are used.
 */
#define FLASH_CAL_MAGIC         0xCAFEBEEFUL

/**
 * @brief Magic number for stored maze wall map.
 */
#define FLASH_MAZE_MAGIC        0xDEADC0DEUL

/* =========================================================================
 * SECTION 13 — OLED DISPLAY  (SSD1306 128×64, optional)
 * =========================================================================
 * Interface: I2C1 shared with MPU6500 (PB8 SCL, PB9 SDA).
 * Set OLED_ENABLED to 0 to strip all OLED code from the build.
 * ======================================================================= */

/**
 * @brief Master OLED enable switch.
 * @details 1 = OLED code compiled in.  0 = all oled_xxx() calls become
 *          empty static inline stubs — zero flash, zero RAM, zero time.
 */
#define OLED_ENABLED            1U

/** SSD1306 7-bit I2C address (SA0 = GND → 0x3C) */
#define OLED_I2C_ADDR           0x3CU

/** Display width in pixels */
#define OLED_WIDTH_PX           128U

/** Display height in pixels */
#define OLED_HEIGHT_PX          64U

/** Number of 8-pixel-tall pages (= height / 8) */
#define OLED_PAGES              8U

/** OLED display refresh interval (ms) — no need to update every 1 ms */
#define OLED_REFRESH_MS         100U

/* =========================================================================
 * SECTION 14 — DIP SWITCH MODE TABLE
 * =========================================================================
 *  SW3 SW2 SW1 SW0   Dec   Mode name               Description
 *  ─── ─── ─── ───   ───   ─────────────────────   ─────────────────────────
 *  OFF OFF OFF OFF     0   MONITOR                 Sensor live-view, UART out
 *  OFF OFF OFF ON      1   IR_CALIBRATE            IR ambient + wall cal
 *  OFF OFF ON  OFF     2   MOTOR_TEST              Spin each motor, verify enc
 *  OFF OFF ON  ON      3   STRAIGHT_TEST           Drive 5 cells, check drift
 *  OFF ON  OFF OFF     4   TURN_TEST               90 / 180 / 360° gyro turns
 *  OFF ON  OFF ON      5   WALL_FOLLOWER           Left-hand rule fallback
 *  OFF ON  ON  OFF     6   SEARCH_RUN              Flood-fill exploration
 *  OFF ON  ON  ON      7   SPEED_RUN_1             SPD_RUN1 (400 mm/s)
 *  ON  OFF OFF OFF     8   SPEED_RUN_2             SPD_RUN2 (600 mm/s)
 *  ON  OFF OFF ON      9   SPEED_RUN_3             SPD_RUN3 (750 mm/s)
 *  ON  OFF ON  OFF    10   AUTO_QUALIFIER          Search → Run1 → Run2 → Run3
 *  ON  OFF ON  ON     11   GYRO_DEBUG              Live yaw + dps on OLED/UART
 *  ON  ON  OFF OFF    12   PRINT_MAZE              Dump wall map via UART
 *  ON  ON  OFF ON     13   BATTERY_CHECK           Display voltage, warn if low
 *  ON  ON  ON  OFF    14   STARTUP_TEST            Full self-test sequence
 *  ON  ON  ON  ON     15   RESERVED                Do not use
 * ======================================================================= */

#define MODE_MONITOR            0U
#define MODE_IR_CALIBRATE       1U
#define MODE_MOTOR_TEST         2U
#define MODE_STRAIGHT_TEST      3U
#define MODE_TURN_TEST          4U
#define MODE_WALL_FOLLOWER      5U
#define MODE_SEARCH_RUN         6U
#define MODE_SPEED_RUN_1        7U
#define MODE_SPEED_RUN_2        8U
#define MODE_SPEED_RUN_3        9U
#define MODE_AUTO_QUALIFIER     10U
#define MODE_GYRO_DEBUG         11U
#define MODE_PRINT_MAZE         12U
#define MODE_BATTERY_CHECK      13U
#define MODE_STARTUP_TEST       14U
#define MODE_RESERVED           15U

/* =========================================================================
 * SECTION 15 — CONTROL LOOP TIMING
 * =========================================================================
 * TIM5 fires the 1 kHz control loop interrupt.
 * TIM5 settings: PSC = 99, ARR = 999
 *   tick rate = 100 MHz / (99+1) = 1 MHz
 *   interrupt  = 1 MHz / (999+1) = 1 kHz
 * ======================================================================= */

/** Control loop frequency (Hz) */
#define CTRL_LOOP_HZ            1000U

/** Control loop period (ms) */
#define CTRL_LOOP_PERIOD_MS     1U

/** Control loop period (seconds) — use in integration/differentiation */
#define CTRL_LOOP_DT            (1.0f / (float)CTRL_LOOP_HZ)

/** TIM5 prescaler value (0-indexed: actual divisor = PSC + 1) */
#define CTRL_TIM_PSC            99U

/** TIM5 auto-reload value */
#define CTRL_TIM_ARR            999U

/* =========================================================================
 * SECTION 16 — SAFETY LIMITS
 * =========================================================================
 * Applied in safety.c and the main motion controller.
 * ======================================================================= */

/**
 * @brief Maximum allowed run time per trial (ms).
 * @details MicroMaze 3 rule: 8 minutes per qualifier trial.
 *          8 × 60 × 1000 = 480 000 ms.  Robot stops itself after this.
 */
#define SAFETY_MAX_RUN_MS       480000UL

/**
 * @brief Stall detection speed threshold (mm/s).
 * @details If measured speed < this for SAFETY_STALL_TIME_MS despite
 *          PWM > SAFETY_STALL_PWM_THRESH, declare a stall.
 */
#define SAFETY_STALL_SPD_MMPS   5.0f

/** Minimum PWM above which stall detection is active */
#define SAFETY_STALL_PWM_THRESH 500U

/**
 * @brief Time below stall threshold before stall is declared (ms).
 */
#define SAFETY_STALL_TIME_MS    500U

/**
 * @brief Maximum single-run distance before forced stop (mm).
 * @details 16 × 16 maze: worst-case path ≈ 256 cells × 180 mm = 46 080 mm.
 *          Set to 60 000 mm as a generous safety margin.
 */
#define SAFETY_MAX_DIST_MM      60000.0f

/** Maximum yaw error during a straight move before emergency stop (deg) */
#define SAFETY_MAX_YAW_DEG      30.0f

/* =========================================================================
 * SECTION 17 — BUZZER TONE DURATIONS
 * =========================================================================
 * All durations in milliseconds.
 * Buzzer is a passive piezo driven from PB1 via a small transistor.
 * ======================================================================= */

/** Short beep — run start acknowledgement */
#define BUZZ_START_MS           50U

/** Double-beep on-time — boot confirmation */
#define BUZZ_BOOT_MS            100U

/** Double-beep off-time between pulses */
#define BUZZ_BOOT_GAP_MS        80U

/** Long beep — goal reached */
#define BUZZ_GOAL_MS            500U

/** Error pattern on-time (×3 long beeps) */
#define BUZZ_ERROR_MS           300U

/** Error pattern off-time between beeps */
#define BUZZ_ERROR_GAP_MS       150U

/** Low-battery rapid triple beep on-time */
#define BUZZ_LOWBATT_MS         60U

/** Low-battery rapid triple beep off-time */
#define BUZZ_LOWBATT_GAP_MS     60U

/* =========================================================================
 * SECTION 18 — LOGGING CONFIGURATION
 * =========================================================================
 * Controls Communication/logger.c — the UART/USB-CDC debug printf wrapper.
 * Log level values match LogLevel_t in logger.h (kept as plain integers
 * here since config.h has zero header dependencies by design).
 * ======================================================================= */

/**
 * @brief Master switch for all LOG_* macros.
 * @details Set to 0U to strip every LOG_DEBUG/INFO/WARN/ERROR/RAW call
 *          out of the build entirely (macros expand to nothing) — use
 *          for the competition build to save Flash and remove the
 *          blocking UART transmit time from the main loop.
 */
#define LOG_ENABLED              1U

/**
 * @brief Default runtime log level at boot (0=DEBUG 1=INFO 2=WARN
 *        3=ERROR 4=NONE). Raise at runtime with logger_set_level()
 *        e.g. to silence DEBUG output during a competition run.
 */
#define LOG_LEVEL_DEFAULT        0U

/**
 * @brief Max characters (excluding NUL) for one formatted log line.
 * @details Longest current call site is the path-optimizer move dump
 *          at ~60 chars; 160 leaves generous headroom. Lines longer
 *          than this are truncated, never overflowed.
 */
#define LOG_BUF_SIZE             160U

/**
 * @brief UART baud rate used by logger.c (USART1, 8N1).
 * @note  PA9 (USART1_TX) conflicts with TIM1_CH2 (motor PWM) — see the
 *        uart1_init() note in main.c. Route to USB CDC instead if that
 *        conflict has not been resolved on your board.
 */
#define LOG_UART_BAUD            115200U

/**
 * @brief HAL_UART_Transmit blocking timeout per log line (ms).
 * @details Generous margin for a 160-byte line at 115200 baud
 *          (~14 ms transmit time) so a busy bus never hangs the
 *          caller indefinitely.
 */
#define LOG_UART_TIMEOUT_MS      50U

/* =========================================================================
 * SECTION 19 — CALIBRATION UX TIMING
 * =========================================================================
 * Controls System/calibration.c — the interactive IR calibration
 * sequence (Mode 1) and the automatic gyro zero-rate calibration that
 * runs at every boot. GYRO_CAL_SAMPLES (Section 10) is the gyro side
 * of this; the constants below are the IR side.
 * ======================================================================= */

/**
 * @brief  Time given to the operator to reposition the robot between
 *         each IR calibration step (ms).
 * @details Step 1 (ir_cal_ambient()) needs the robot in open space,
 *          Step 2 (ir_cal_wall()) needs it in the start cell against
 *          three walls at 90 mm. No button-press confirmation exists
 *          yet (Drivers/buttons.c is not built), so calibration.c uses
 *          this fixed delay after each OLED prompt instead of waiting
 *          for an acknowledgement.
 */
#define CAL_IR_STEP_DELAY_MS     3000U

/**
 * @brief  Extra settle time after positioning, immediately before each
 *         capture (ms).
 * @details The IR DMA scan (ir.c) runs continuously in the background,
 *          so this is a small margin for the operator's hand to be
 *          clear of the robot and for readings to stabilise, not a
 *          hardware requirement.
 */
#define CAL_IR_SETTLE_MS         200U

/* =========================================================================
 * SECTION 20 — STARTUP SELF-TEST PARAMETERS
 * =========================================================================
 * Controls System/startup_test.c (DIP Mode 14). Drives raw PWM directly
 * via motors_set() — deliberately bypasses motion.c/PID so the test
 * exercises the motor/encoder hardware path independently of the
 * control loop it would otherwise be validating.
 * ======================================================================= */

/**
 * @brief  PWM magnitude used for the brief motor/encoder spin test.
 * @details ~15 % of PWM_MAX — enough for the wheels to turn measurably
 *          against static friction without the robot travelling far
 *          on a bench in STARTUP_TEST_MOTOR_MS.
 */
#define STARTUP_TEST_PWM         ((int32_t)(PWM_MAX * 0.15f))

/** Duration of the motor/encoder spin test (ms). */
#define STARTUP_TEST_MOTOR_MS    300U

/**
 * @brief  Minimum |encoder delta| (counts) over STARTUP_TEST_MOTOR_MS
 *         for a wheel to be considered "moving" — set low to avoid a
 *         false FAIL from a slow start under load, while still catching
 *         a genuinely disconnected encoder or seized gearbox.
 */
#define STARTUP_TEST_MIN_ENC_COUNTS  20

/* =========================================================================
 * SECTION 21 — BUTTON DEBOUNCE
 * =========================================================================
 * Controls Drivers/buttons.c — debounce and edge detection on top of
 * the raw BTN_PRESSED() macro in pins.h (PC15, the single onboard
 * test/user button — separate from the 4-position DIP mode selector).
 * ======================================================================= */

/**
 * @brief  Time the raw pin state must hold steady before a press or
 *         release is accepted as real (ms).
 * @details Typical tact-switch mechanical bounce settles within a few
 *          milliseconds; 30 ms leaves generous margin without making
 *          the button feel laggy to a human press.
 */
#define BTN_DEBOUNCE_MS          30U

/* =========================================================================
 * SECTION 22 — DERIVED CONSTANTS  (do not edit — computed from above)
 * =========================================================================
 * These are computed at compile time from the base constants above.
 * They are provided as named constants so call sites remain readable.
 * ======================================================================= */

/**
 * @brief Maximum robot linear speed (mm/s).
 * @details At 100 % PWM, no-load output shaft speed drives the wheel at:
 *          750 RPM / 60 s × π × 34 mm = 1335 mm/s
 *          This matches SPD_MAX_THEORETICAL and is used for range checks.
 */
#define SPD_ABSOLUTE_MAX        SPD_MAX_THEORETICAL

/**
 * @brief Distance the robot travels per control loop tick at search speed (mm).
 * @details SPD_SEARCH × CTRL_LOOP_DT = 200 × 0.001 = 0.2 mm per tick
 */
#define DIST_PER_TICK_SEARCH    (SPD_SEARCH * CTRL_LOOP_DT)

/**
 * @brief Number of encoder counts in one cell at constant speed.
 * @details counts = CELL_WIDTH_MM / MM_PER_COUNT
 *          = 180.0 / 0.12716 ≈ 1415 counts
 */
#define COUNTS_PER_CELL         ((uint32_t)(CELL_WIDTH_MM / MM_PER_COUNT))

/**
 * @brief Distance to begin braking from search speed to stop (mm).
 * @details s = v² / (2 × a)
 *          = 200² / (2 × 3000) = 40 000 / 6000 = 6.67 mm
 */
#define BRAKE_DIST_SEARCH_MM    (SPD_SEARCH * SPD_SEARCH / (2.0f * DECEL_SEARCH))

/**
 * @brief Distance to begin braking from SPD_RUN1 (mm).
 */
#define BRAKE_DIST_RUN1_MM      (SPD_RUN1 * SPD_RUN1 / (2.0f * DECEL_NORMAL))

/**
 * @brief Distance to begin braking from SPD_RUN2 (mm).
 */
#define BRAKE_DIST_RUN2_MM      (SPD_RUN2 * SPD_RUN2 / (2.0f * DECEL_NORMAL))

/**
 * @brief Distance to begin braking from SPD_RUN3 (mm).
 */
#define BRAKE_DIST_RUN3_MM      (SPD_RUN3 * SPD_RUN3 / (2.0f * DECEL_NORMAL))

/**
 * @brief PWM value at which the motor is guaranteed to move (minimum effort).
 * @details Empirically ~8 % of PWM_MAX for N20 motors.
 */
#define PWM_DEADBAND            400U

/* =========================================================================
 * SECTION 23 — COMPILE-TIME ASSERTIONS
 * =========================================================================
 * These fire at compile time if any derived constant is nonsensical.
 * They cost zero Flash and zero RAM.
 * ======================================================================= */

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)

/** Encoder CPR must equal 840 for this hardware */
_Static_assert(COUNTS_PER_REV == 840U,
    "COUNTS_PER_REV must be 840 (7 PPR × 4 quad × 30:1 gear)");

/** PWM ARR must produce 20 kHz from 100 MHz APB2 timer clock */
_Static_assert((APB2_TIM_CLK_HZ / (PWM_ARR + 1U)) == PWM_FREQUENCY_HZ,
    "PWM_ARR does not produce PWM_FREQUENCY_HZ from APB2_TIM_CLK_HZ");

/** TIM5 must produce CTRL_LOOP_HZ from APB1 timer clock */
_Static_assert(
    (APB1_TIM_CLK_HZ / ((CTRL_TIM_PSC + 1U) * (CTRL_TIM_ARR + 1U)))
    == CTRL_LOOP_HZ,
    "TIM5 PSC/ARR does not produce CTRL_LOOP_HZ from APB1_TIM_CLK_HZ");

/** Goal cells must be inside the maze bounds */
_Static_assert(GOAL_ROW_MAX < MAZE_SIZE && GOAL_COL_MAX < MAZE_SIZE,
    "Goal cells must be within MAZE_SIZE x MAZE_SIZE grid");

/** Start cell must be inside the maze bounds */
_Static_assert(START_ROW < MAZE_SIZE && START_COL < MAZE_SIZE,
    "Start cell must be within MAZE_SIZE x MAZE_SIZE grid");

/** OLED_PAGES must equal OLED_HEIGHT_PX / 8 */
_Static_assert(OLED_PAGES == OLED_HEIGHT_PX / 8U,
    "OLED_PAGES must equal OLED_HEIGHT_PX / 8");

/** Motor polarity must be +1 or -1 */
_Static_assert(LEFT_MOTOR_POL  ==  1 || LEFT_MOTOR_POL  == -1,
    "LEFT_MOTOR_POL must be +1 or -1");
_Static_assert(RIGHT_MOTOR_POL ==  1 || RIGHT_MOTOR_POL == -1,
    "RIGHT_MOTOR_POL must be +1 or -1");

#endif /* C11 static_assert */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
