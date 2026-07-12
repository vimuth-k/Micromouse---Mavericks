/**
 * @file config.h
 * @brief Central, hardware-independent tuning constants for the Micromouse.
 *
 * Edit the calibrated values in this file when tuning the completed robot.
 * Pin assignments, peripheral handles, HAL definitions, and executable code
 * must not be placed here.
 *
 * @copyright Copyright (c) 2026
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================== */
/* PHYSICAL ROBOT                                                             */
/* ========================================================================== */

#define PI_F                        (3.14159265358979323846f) /**< Single-precision pi constant. */
#define WHEEL_DIAMETER_MM           (34.0f) /**< Nominal driven-wheel diameter. */
#define WHEEL_SPACING_MM            (82.0f) /**< Centre-to-centre wheel spacing. */
#define GEAR_RATIO                  (30U)   /**< Motor revolutions per wheel revolution. */
#define ENCODER_PPR                 (7U)    /**< Encoder pulses per motor-shaft revolution. */
#define ENCODER_QUADRATURE_FACTOR   (4U)    /**< Timer encoder-interface decoding multiplier. */
#define COUNTS_PER_REV              \
    (ENCODER_PPR * ENCODER_QUADRATURE_FACTOR * GEAR_RATIO) /**< Counts per wheel revolution. */
#define MM_PER_COUNT                \
    ((PI_F * WHEEL_DIAMETER_MM) / (float)(COUNTS_PER_REV)) /**< Encoder distance scale. */

/* ========================================================================== */
/* PWM                                                                         */
/* ========================================================================== */

#define PWM_FREQUENCY_HZ            (20000U) /**< Motor PWM switching frequency. */
#define PWM_TIMER_ARR               (4999U)  /**< Timer auto-reload value for motor PWM. */
#define PWM_MAX                     (4999U)  /**< Maximum motor PWM compare value. */
#define PWM_MIN                     (0U)     /**< Minimum motor PWM compare value. */

/* ========================================================================== */
/* MAZE                                                                        */
/* ========================================================================== */

#define MAZE_SIZE                   (16U)    /**< Number of cells per maze axis. */
#define CELL_WIDTH_MM               (180.0f) /**< Standard maze-cell width. */
#define HALF_CELL_MM                (CELL_WIDTH_MM / 2.0f) /**< Cell-centre distance from a wall. */
#define GOAL_ROW_MIN                (7U)     /**< First row of the central 2x2 goal. */
#define GOAL_ROW_MAX                (8U)     /**< Last row of the central 2x2 goal. */
#define GOAL_COL_MIN                (7U)     /**< First column of the central 2x2 goal. */
#define GOAL_COL_MAX                (8U)     /**< Last column of the central 2x2 goal. */
#define MAZE_UNREACHABLE_COST       (255U)   /**< Flood-fill cost for an unreached cell. */

/* ========================================================================== */
/* CALIBRATION DEFAULTS                                                        */
/* ========================================================================== */

#define LEFT_MOTOR_POLARITY         (1)      /**< Motor direction multiplier; change after test if required. */
#define RIGHT_MOTOR_POLARITY        (-1)     /**< Motor direction multiplier; change after test if required. */
#define LEFT_ENCODER_POLARITY       (1)      /**< Encoder count direction multiplier; change after test if required. */
#define RIGHT_ENCODER_POLARITY      (-1)     /**< Encoder count direction multiplier; change after test if required. */
#define IR_THRESHOLD_FRONT          (450U)   /**< Safe fallback front-wall ADC threshold. */
#define IR_THRESHOLD_SIDE           (400U)   /**< Safe fallback side-wall ADC threshold. */
#define IR_THRESHOLD_DIAGONAL       (350U)   /**< Safe fallback diagonal-wall ADC threshold. */
#define IR_FRONT_CLOSE_THRESHOLD    (700U)   /**< ADC threshold for close front-wall alignment. */
#define IR_AVERAGE_SAMPLES          (4U)     /**< ADC samples averaged per IR measurement. */
#define ADC_REFERENCE_VOLTAGE       (3.300f) /**< ADC reference voltage used in conversions. */
#define ADC_MAX_COUNT               (4095U)  /**< Maximum value from the 12-bit ADC. */
#define FLASH_CALIBRATION_ADDRESS   (0x08060000UL) /**< Reserved flash address for calibration data. */
#define FLASH_CALIBRATION_MAGIC     (0xDEADBEEFUL) /**< Validation signature for calibration data. */

/* ========================================================================== */
/* SPEED AND ACCELERATION PROFILES                                            */
/* ========================================================================== */

#define SPEED_SEARCH_MMPS           (200.0f) /**< Conservative exploration speed. */
#define SPEED_RETURN_MMPS           (250.0f) /**< Return-to-start speed. */
#define SPEED_RUN_1_MMPS            (400.0f) /**< First validated speed-run profile. */
#define SPEED_RUN_2_MMPS            (600.0f) /**< Second validated speed-run profile. */
#define SPEED_RUN_3_MMPS            (750.0f) /**< Competition speed-run profile. */
#define SPEED_MAX_MMPS              (800.0f) /**< Absolute configured linear speed limit. */
#define ACCEL_SEARCH_MMPS2          (2000.0f) /**< Exploration acceleration limit. */
#define DECEL_SEARCH_MMPS2          (3000.0f) /**< Exploration deceleration limit. */
#define ACCEL_NORMAL_MMPS2          (3000.0f) /**< Normal-run acceleration limit. */
#define DECEL_NORMAL_MMPS2          (4000.0f) /**< Normal-run deceleration limit. */
#define TURN_SPEED_DPS              (360.0f) /**< Nominal in-place turn speed. */
#define TURN_ACCEL_DPS2             (3600.0f) /**< In-place turn acceleration limit. */

/* ========================================================================== */
/* PID DEFAULTS                                                                */
/* ========================================================================== */

/* All control gains assume a 1 kHz control update. Tune Kp, then Ki, then Kd. */
#define KP_SPEED                    (4.5f)   /**< Per-wheel speed-loop proportional gain. */
#define KI_SPEED                    (9.0f)   /**< Per-wheel speed-loop integral gain. */
#define KD_SPEED                    (0.05f)  /**< Per-wheel speed-loop derivative gain. */
#define SPEED_INTEGRAL_LIMIT        (2000.0f) /**< Speed-loop integral clamp. */
#define KP_STRAIGHT                 (2.5f)   /**< Encoder straightness-loop proportional gain. */
#define KI_STRAIGHT                 (0.0f)   /**< Encoder straightness-loop integral gain. */
#define KD_STRAIGHT                 (0.4f)   /**< Encoder straightness-loop derivative gain. */
#define STRAIGHT_INTEGRAL_LIMIT     (500.0f) /**< Straightness-loop integral clamp. */
#define KP_HEADING                  (1.8f)   /**< Gyro heading-loop proportional gain. */
#define KI_HEADING                  (0.0f)   /**< Gyro heading-loop integral gain. */
#define KD_HEADING                  (0.1f)   /**< Gyro heading-loop derivative gain. */
#define HEADING_INTEGRAL_LIMIT      (45.0f)  /**< Heading-loop integral clamp. */
#define KP_WALL_CENTER              (0.35f)  /**< Side-wall-centering proportional gain. */
#define KI_WALL_CENTER              (0.0f)   /**< Side-wall-centering integral gain. */
#define KD_WALL_CENTER              (0.0f)   /**< Side-wall-centering derivative gain. */
#define KP_TURN                     (12.0f)  /**< Gyro turn-loop proportional gain. */
#define KI_TURN                     (0.0f)   /**< Gyro turn-loop integral gain. */
#define KD_TURN                     (0.3f)   /**< Gyro turn-loop derivative gain. */
#define TURN_INTEGRAL_LIMIT         (45.0f)  /**< Turn-loop integral clamp. */
#define TURN_TOLERANCE_DEG          (0.8f)   /**< Turn completion angular tolerance. */
#define TURN_SETTLE_MS              (30U)    /**< Delay after a completed turn. */

/* ========================================================================== */
/* IMU                                                                         */
/* ========================================================================== */

#define GYRO_FULL_SCALE_DPS          (500.0f) /**< Configured gyroscope full-scale range. */
#define GYRO_SENSITIVITY_LSB_PER_DPS (65.5f)  /**< Gyroscope scale at the configured full scale. */
#define GYRO_CALIBRATION_SAMPLES    (500U)   /**< Stationary samples collected for gyro bias. */
#define GYRO_SAMPLE_PERIOD_S        (0.001f) /**< Gyro integration period at control rate. */
#define GYRO_MAX_BIAS_DPS           (5.0f)   /**< Maximum accepted stationary gyro bias. */

/* ========================================================================== */
/* BATTERY                                                                     */
/* ========================================================================== */

#define BATTERY_NOMINAL_VOLTAGE_V   (7.4f)   /**< Nominal voltage of the 2S LiPo pack. */
#define BATTERY_FULL_VOLTAGE_V      (8.4f)   /**< Fully charged 2S LiPo voltage. */
#define BATTERY_LOW_VOLTAGE_V       (6.8f)   /**< Low-battery warning threshold. */
#define BATTERY_CRITICAL_VOLTAGE_V  (6.4f)   /**< Motion-inhibit battery threshold. */
#define BATTERY_VOLTAGE_DIVIDER_RATIO (2.0f) /**< Battery monitor divider reconstruction ratio. */
#define BATTERY_ADC_SCALE_V_PER_COUNT \
    ((ADC_REFERENCE_VOLTAGE / (float)(ADC_MAX_COUNT)) * BATTERY_VOLTAGE_DIVIDER_RATIO) /**< ADC count-to-pack-voltage scale. */
#define BATTERY_FILTER_SAMPLES      (8U)     /**< ADC samples averaged for battery measurements. */

/* ========================================================================== */
/* SAFETY                                                                      */
/* ========================================================================== */

#define SAFETY_MAX_RUN_MS           (480000UL) /**< Maximum continuous run time. */
#define SAFETY_STALL_TIMEOUT_MS     (500U)     /**< Motor stall detection timeout. */
#define SAFETY_STALL_SPEED_MMPS     (5.0f)     /**< Speed below which a driven wheel is stalled. */
#define SAFETY_MAX_WHEEL_SPEED_MMPS (SPEED_MAX_MMPS) /**< Maximum permitted wheel speed. */
#define SAFETY_MAX_YAW_RATE_DPS     (900.0f)   /**< Maximum permitted yaw rate. */
#define SAFETY_SENSOR_FAULT_COUNT   (10U)      /**< Invalid readings allowed before sensor fault. */

/* ========================================================================== */
/* SYSTEM TIMING                                                               */
/* ========================================================================== */

#define CONTROL_LOOP_HZ             (1000U) /**< Main control-loop frequency. */
#define CONTROL_LOOP_PERIOD_MS      (1U)    /**< Main control-loop period. */
#define SENSOR_PERIOD_MS            (2U)    /**< IR sensor acquisition period. */
#define SENSOR_SETTLE_US            (60U)   /**< IR emitter-on settling delay. */
#define NAVIGATION_PERIOD_MS        (10U)   /**< Navigation and flood-fill update period. */
#define BATTERY_PERIOD_MS           (100U)  /**< Battery monitor update period. */
#define DISPLAY_PERIOD_MS           (100U)  /**< Display update period. */
#define SCHEDULER_TICK_MS           (1U)    /**< Cooperative scheduler resolution. */

/* ========================================================================== */
/* USER INTERFACE                                                              */
/* ========================================================================== */

#define OLED_ENABLED                (1U)    /**< Enables OLED status display support. */
#define BUZZ_BOOT_MS                 (100U)  /**< Startup acknowledgement tone duration. */
#define BUZZ_GOAL_MS                 (500U)  /**< Goal-reached tone duration. */
#define BUZZ_ERROR_MS                (1000U) /**< Fault indication tone duration. */

#if (CONTROL_LOOP_HZ != 1000U)
#error "PID tuning assumes a 1 kHz control loop."
#endif

#endif /* CONFIG_H */

