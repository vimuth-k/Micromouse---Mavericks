#ifndef CONFIG_H
#define CONFIG_H

/*
 * ================================================================
 *  MICROMOUSE  ·  config.h
 *  All tunable robot constants in one place.
 *  !! Edit ONLY this file to tune your robot !!
 * ================================================================
 */

/* ── PHYSICAL ROBOT ─────────────────────────────────────────────────── */
#define WHEEL_DIAMETER_MM       34.0f
#define WHEEL_SPACING_MM        82.0f   /* axle-to-axle — MEASURE YOURS  */
#define GEAR_RATIO              30.0f   /* 6V 750RPM motor, 30:1 gearbox */
#define ENCODER_PPR             7       /* pulses per motor shaft rev     */
/* Quadrature × PPR × gear = counts per wheel revolution                 */
#define COUNTS_PER_REV          (ENCODER_PPR * 4 * (int)GEAR_RATIO)    /* 840 */
#define MM_PER_COUNT            (3.14159265f * WHEEL_DIAMETER_MM / (float)COUNTS_PER_REV)
/* = π × 34 / 840 = 0.12723 mm/count                                    */

/* ── MAZE (MicroMaze 3) ──────────────────────────────────────────────── */
#define MAZE_SIZE               16
#define CELL_WIDTH_MM           180.0f
#define HALF_CELL_MM            90.0f
#define GOAL_ROW_MIN            7
#define GOAL_ROW_MAX            8
#define GOAL_COL_MIN            7
#define GOAL_COL_MAX            8

/* ── MOTOR POLARITY  (set after Stage-B motor test) ─────────────────── */
/* +1 = correct,  -1 = flip direction in software                        */
#define LEFT_MOTOR_POL          1
#define RIGHT_MOTOR_POL        -1   /* mirrors on opposite side           */
#define LEFT_ENC_POL            1
#define RIGHT_ENC_POL          -1

/* ── PWM  (TIM1 at 100 MHz) ─────────────────────────────────────────── */
/* 20 kHz: PSC=0, ARR=4999 → 100 000 000 / 5000 = 20 000 Hz             */
#define PWM_ARR                 4999
#define PWM_MAX                 4999
#define PWM_MIN                 0

/* ── SPEED PROFILES (mm/s) ───────────────────────────────────────────── */
#define SPD_SEARCH              200.0f
#define SPD_RETURN              250.0f
#define SPD_RUN1                400.0f
#define SPD_RUN2                600.0f
#define SPD_RUN3                750.0f
#define SPD_MAX                 800.0f

/* ── ACCELERATION PROFILES (mm/s²) ──────────────────────────────────── */
#define ACCEL_NORMAL           3000.0f
#define DECEL_NORMAL           4000.0f
#define ACCEL_SEARCH           2000.0f
#define DECEL_SEARCH           3000.0f

/* ── PID GAINS ───────────────────────────────────────────────────────── */
/* All PIDs run at 1 kHz; gains are per-sample (not per-second).         */
/* Tune in this order: Kp → Ki → Kd. Start small, increase slowly.      */

/* Speed PID (per wheel) */
#define KP_SPEED                4.5f
#define KI_SPEED                9.0f
#define KD_SPEED                0.05f
#define SPEED_INTEG_LIMIT       2000.0f

/* Straightness PID (encoder count difference) */
#define KP_STRAIGHT             2.5f
#define KI_STRAIGHT             0.0f
#define KD_STRAIGHT             0.4f
#define STRAIGHT_INTEG_LIMIT    500.0f

/* Heading PID (gyro yaw correction during straights) */
#define KP_HEADING              1.8f
#define KI_HEADING              0.0f
#define KD_HEADING              0.1f
#define HEADING_INTEG_LIMIT     45.0f

/* Wall-centering P gain (side IR sensors) */
#define KP_WALL_CENTER          0.35f

/* Turn PID (gyro-based) */
#define KP_TURN                 12.0f
#define KI_TURN                 0.0f
#define KD_TURN                 0.3f
#define TURN_MIN_PWM            300.0f   /* min PWM to actually move      */
#define TURN_MAX_PWM            2500.0f
#define TURN_TOLERANCE_DEG      0.8f     /* stop when error < this        */
#define TURN_SETTLE_MS          30       /* wait after turn completes     */

/* ── IR SENSOR THRESHOLDS (filled by calibration routine) ───────────── */
/* These are safe fallback defaults. Replace after Mode-1 calibration.   */
#define IR_THRESH_FRONT         450
#define IR_THRESH_SIDE          400
#define IR_THRESH_DIAG          350
#define IR_FRONT_CLOSE          700     /* triggers wall alignment        */

/* ── MPU6500 ─────────────────────────────────────────────────────────── */
#define MPU_GYRO_FS_500DPS      0x08    /* ±500 dps, 65.5 LSB/(deg/s)    */
#define GYRO_SENSITIVITY        65.5f   /* LSB per deg/s at ±500 dps     */
#define GYRO_CAL_SAMPLES        500     /* samples at startup calibration */
#define GYRO_DT                 0.001f  /* 1 ms per control tick          */

/* ── SYSTEM TIMING ───────────────────────────────────────────────────── */
#define CTRL_LOOP_HZ            1000    /* TIM5 interrupt rate            */
#define SENSOR_SETTLE_US        60      /* emitter ON settle time         */

/* ── BATTERY ─────────────────────────────────────────────────────────── */
/* 2S LiPo: nominal 7.4V, cutoff 6.8V, full 8.4V                        */
/* Voltage divider 10k/10k: Vbat/2 → ADC; ADC ref = 3.3V, 12-bit       */
/* ADC_to_V = raw * 3.3 / 4095 * 2                                      */
#define BATT_LOW_VOLTAGE        6.8f
#define BATT_CRITICAL_VOLTAGE   6.4f
#define BATT_ADC_SCALE          (3.3f / 4095.0f * 2.0f)

/* ── FLASH STORAGE ───────────────────────────────────────────────────── */
/* STM32F411: sector 7 = 0x08060000 (128 KB). Used for calibration.     */
#define FLASH_CALIB_SECTOR      FLASH_SECTOR_7
#define FLASH_CALIB_ADDR        0x08060000UL
#define FLASH_CALIB_MAGIC       0xDEADBEEFUL

/* ── OLED ────────────────────────────────────────────────────────────── */
#define OLED_I2C_ADDR           0x3C
#define OLED_ENABLED            1       /* set 0 to disable all OLED code */

/* ── BUZZER TONES ────────────────────────────────────────────────────── */
#define BUZZ_BOOT_MS            100
#define BUZZ_RUN_START_MS       50
#define BUZZ_GOAL_MS            500
#define BUZZ_ERROR_MS           1000

/* ── SAFETY ─────────────────────────────────────────────────────────────*/
#define SAFETY_MAX_RUN_MS       480000UL /* 8 minutes = max run time      */
#define SAFETY_STALL_MS         500      /* motor stall detection timeout */
#define SAFETY_STALL_SPEED_MMP  5.0f    /* below this = stall (mm/s)     */

/* ── SCHEDULER ──────────────────────────────────────────────────────────*/
#define SCHED_TICK_MS           1        /* scheduler resolution          */

/* ── DIP SWITCH MODE TABLE ───────────────────────────────────────────── */
/*
 *  SW3 SW2 SW1 SW0  │  Mode  │  Description
 *  ─────────────────┼────────┼─────────────────────────────────────────
 *  OFF OFF OFF OFF  │   0    │  Sensor monitor / safe boot
 *  OFF OFF OFF ON   │   1    │  IR calibration (ambient + wall)
 *  OFF OFF ON  OFF  │   2    │  Motor & encoder test
 *  OFF OFF ON  ON   │   3    │  Straight-line PID test (5 cells)
 *  OFF ON  OFF OFF  │   4    │  Gyro turn test (90/180/360)
 *  OFF ON  OFF ON   │   5    │  Wall-follower fallback
 *  OFF ON  ON  OFF  │   6    │  SEARCH RUN (flood-fill exploration)
 *  OFF ON  ON  ON   │   7    │  SPEED RUN 1  (400 mm/s)
 *  ON  OFF OFF OFF  │   8    │  SPEED RUN 2  (600 mm/s)
 *  ON  OFF OFF ON   │   9    │  SPEED RUN 3  (750 mm/s — MAX)
 *  ON  OFF ON  OFF  │  10    │  Auto qualifier (search → 3 speed runs)
 *  ON  OFF ON  ON   │  11    │  Gyro live display / debug
 *  ON  ON  OFF OFF  │  12    │  Print maze map via UART
 *  ON  ON  OFF ON   │  13    │  Battery voltage check
 *  ON  ON  ON  OFF  │  14    │  Full system startup test
 *  ON  ON  ON  ON   │  15    │  Reserved
 */

#endif /* CONFIG_H */
