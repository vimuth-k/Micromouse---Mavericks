/**
 * @file types.h
 * @brief Common, hardware-independent types for the Micromouse firmware.
 *
 * This header defines the shared state, measurement, calibration, motion,
 * and maze data types used across firmware subsystems. It contains no driver
 * interfaces, hardware-register definitions, or executable code.
 *
 * @copyright Copyright (c) 2026
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Number of IR emitter and receiver pairs fitted to the robot. */
#define IR_SENSOR_COUNT                (6U)

/** @brief Number of individually controlled IR emitters fitted to the robot. */
#define IR_EMITTER_COUNT               IR_SENSOR_COUNT

/* ========================================================================== */
/* Robot operating modes and states                                            */
/* ========================================================================== */

/**
 * @brief User-selectable high-level operating mode.
 */
typedef enum
{
    ROBOT_MODE_SAFE_BOOT = 0U, /**< Safe boot with actuator outputs disabled. */
    ROBOT_MODE_SENSOR_MONITOR, /**< Live sensor monitoring mode. */
    ROBOT_MODE_IR_CALIBRATION, /**< IR emitter and receiver calibration mode. */
    ROBOT_MODE_MOTOR_TEST, /**< Motor and encoder verification mode. */
    ROBOT_MODE_STRAIGHT_TEST, /**< Straight-line control test mode. */
    ROBOT_MODE_TURN_TEST, /**< Gyroscope turn-control test mode. */
    ROBOT_MODE_WALL_FOLLOW, /**< Wall-following diagnostic mode. */
    ROBOT_MODE_SEARCH, /**< Flood-fill maze exploration mode. */
    ROBOT_MODE_SPEED_RUN, /**< Optimized maze speed-run mode. */
    ROBOT_MODE_BATTERY_CHECK, /**< Battery measurement display mode. */
    ROBOT_MODE_SYSTEM_TEST /**< Full startup self-test mode. */
} RobotMode;

/**
 * @brief Current top-level execution state of the robot.
 */
typedef enum
{
    ROBOT_STATE_RESET = 0U, /**< Reset state before initialization. */
    ROBOT_STATE_INITIALIZING, /**< Peripheral and subsystem initialization. */
    ROBOT_STATE_READY, /**< Ready and waiting for a start command. */
    ROBOT_STATE_CALIBRATING, /**< Performing a calibration operation. */
    ROBOT_STATE_RUNNING, /**< Actively executing the selected mode. */
    ROBOT_STATE_PAUSED, /**< Motion paused while retaining state. */
    ROBOT_STATE_COMPLETE, /**< Requested operation completed successfully. */
    ROBOT_STATE_FAULT /**< A non-recoverable fault has stopped operation. */
} RobotState;

/**
 * @brief State of an individual managed subsystem.
 */
typedef enum
{
    SUBSYSTEM_STATE_UNINITIALIZED = 0U, /**< Subsystem has not been initialized. */
    SUBSYSTEM_STATE_INITIALIZING, /**< Subsystem initialization is in progress. */
    SUBSYSTEM_STATE_READY, /**< Subsystem is ready for use. */
    SUBSYSTEM_STATE_DEGRADED, /**< Subsystem is operating with a warning. */
    SUBSYSTEM_STATE_FAULT /**< Subsystem is unavailable because of a fault. */
} SubsystemState;

/**
 * @brief Consolidated runtime status for the robot and its principal subsystems.
 */
typedef struct
{
    RobotMode mode; /**< Current user-selected operating mode. */
    RobotState state; /**< Current top-level execution state. */
    SubsystemState motor; /**< Motor driver and encoder subsystem status. */
    SubsystemState imu; /**< IMU subsystem status. */
    SubsystemState sensors; /**< IR sensor subsystem status. */
    SubsystemState battery; /**< Battery-monitor subsystem status. */
    SubsystemState maze; /**< Maze-navigation subsystem status. */
} SystemStatus;

/* ========================================================================== */
/* Navigation and maze types                                                   */
/* ========================================================================== */

/**
 * @brief Cardinal direction in the maze coordinate frame.
 */
typedef enum
{
    DIRECTION_NORTH = 0U, /**< Positive maze-row direction. */
    DIRECTION_EAST, /**< Positive maze-column direction. */
    DIRECTION_SOUTH, /**< Negative maze-row direction. */
    DIRECTION_WEST, /**< Negative maze-column direction. */
    DIRECTION_INVALID /**< Invalid or uninitialized direction. */
} Direction;

/**
 * @brief Discrete turn command relative to the current heading.
 */
typedef enum
{
    TURN_TYPE_NONE = 0U, /**< Maintain the current heading. */
    TURN_TYPE_LEFT_45, /**< Turn 45 degrees to the left. */
    TURN_TYPE_RIGHT_45, /**< Turn 45 degrees to the right. */
    TURN_TYPE_LEFT_90, /**< Turn 90 degrees to the left. */
    TURN_TYPE_RIGHT_90, /**< Turn 90 degrees to the right. */
    TURN_TYPE_UTURN /**< Turn 180 degrees. */
} TurnType;

/**
 * @brief Observed state of a maze wall.
 */
typedef enum
{
    WALL_STATE_UNKNOWN = 0U, /**< Wall has not yet been observed. */
    WALL_STATE_OPEN, /**< No wall is present. */
    WALL_STATE_PRESENT /**< A wall is present. */
} WallState;

/** @brief Bit field identifying one or more sides of a maze cell. */
typedef uint8_t WallMask;

/**
 * @brief Individual bit values used with WallMask.
 */
enum
{
    WALL_MASK_NONE = 0U, /**< No cell sides selected. */
    WALL_MASK_NORTH = (1U << 0U), /**< North cell side. */
    WALL_MASK_EAST = (1U << 1U), /**< East cell side. */
    WALL_MASK_SOUTH = (1U << 2U), /**< South cell side. */
    WALL_MASK_WEST = (1U << 3U) /**< West cell side. */
};

/**
 * @brief One cell in the internal maze representation.
 */
typedef struct
{
    uint8_t wall_mask; /**< Bitwise OR of known wall sides from WallMask. */
    uint8_t open_mask; /**< Bitwise OR of observed open sides from WallMask. */
    uint8_t flood_cost; /**< Flood-fill distance cost. */
    bool visited; /**< True after the robot has entered this cell. */
} Cell;

/* ========================================================================== */
/* Motion and odometry types                                                   */
/* ========================================================================== */

/**
 * @brief Identifier of a drive wheel and its associated motor and encoder.
 */
typedef enum
{
    WHEEL_LEFT = 0U, /**< Left drive wheel. */
    WHEEL_RIGHT /**< Right drive wheel. */
} WheelId;

/**
 * @brief State of one motor command channel.
 */
typedef enum
{
    MOTOR_STATE_DISABLED = 0U, /**< Motor driver channel is disabled. */
    MOTOR_STATE_BRAKE, /**< Motor terminals are electrically braked. */
    MOTOR_STATE_COAST, /**< Motor terminals are released to coast. */
    MOTOR_STATE_FORWARD, /**< Motor is driven in the forward direction. */
    MOTOR_STATE_REVERSE /**< Motor is driven in the reverse direction. */
} MotorState;

/**
 * @brief Cartesian position expressed in millimetres.
 */
typedef struct
{
    float x_mm; /**< Position along the maze x axis. */
    float y_mm; /**< Position along the maze y axis. */
} Position;

/**
 * @brief Linear and angular velocity of the robot.
 */
typedef struct
{
    float linear_mmps; /**< Forward linear velocity in millimetres per second. */
    float angular_dps; /**< Yaw angular velocity in degrees per second. */
} Velocity;

/**
 * @brief Planar pose in the maze coordinate frame.
 */
typedef struct
{
    Position position; /**< Cartesian position. */
    float heading_deg; /**< Heading angle, clockwise from maze north. */
} Pose;

/**
 * @brief Command and feedback data for one motor.
 */
typedef struct
{
    MotorState state; /**< Current requested motor state. */
    float command; /**< Normalized signed motor command in the range [-1.0, 1.0]. */
    float target_speed_mmps; /**< Requested wheel speed. */
    float measured_speed_mmps; /**< Measured wheel speed. */
} MotorData;

/**
 * @brief Quadrature encoder measurement data for one wheel.
 */
typedef struct
{
    int32_t count; /**< Accumulated signed encoder count. */
    int32_t delta_count; /**< Signed count change during the last update. */
    float distance_mm; /**< Integrated wheel travel. */
    float speed_mmps; /**< Estimated wheel speed. */
    uint32_t timestamp_ms; /**< Timestamp of the latest update. */
} EncoderData;

/* ========================================================================== */
/* Sensor and power types                                                      */
/* ========================================================================== */

/**
 * @brief Identifiers for each IR emitter and receiver pair.
 */
typedef enum
{
    IR_SENSOR_FRONT_LEFT = 0U, /**< Front-left sensor pair. */
    IR_SENSOR_FRONT_RIGHT, /**< Front-right sensor pair. */
    IR_SENSOR_SIDE_LEFT, /**< Left-side sensor pair. */
    IR_SENSOR_SIDE_RIGHT, /**< Right-side sensor pair. */
    IR_SENSOR_DIAG_LEFT, /**< Left diagonal sensor pair. */
    IR_SENSOR_DIAG_RIGHT /**< Right diagonal sensor pair. */
} IRSensorId;

/**
 * @brief Measurement and interpretation of one IR receiver.
 */
typedef struct
{
    uint16_t ambient_adc; /**< ADC reading with the associated emitter disabled. */
    uint16_t illuminated_adc; /**< ADC reading with the associated emitter enabled. */
    uint16_t differential_adc; /**< Ambient-subtracted reflected-light reading. */
    float distance_mm; /**< Calibrated estimated wall distance. */
    bool wall_detected; /**< True when the calibrated wall threshold is exceeded. */
    bool valid; /**< True when the measurement passed plausibility checks. */
} IRSensorData;

/**
 * @brief Gyroscope and accelerometer measurements used for attitude estimation.
 */
typedef struct
{
    float gyro_x_dps; /**< Gyroscope x-axis angular rate. */
    float gyro_y_dps; /**< Gyroscope y-axis angular rate. */
    float gyro_z_dps; /**< Gyroscope z-axis angular rate. */
    float accel_x_g; /**< Accelerometer x-axis measurement. */
    float accel_y_g; /**< Accelerometer y-axis measurement. */
    float accel_z_g; /**< Accelerometer z-axis measurement. */
    float heading_deg; /**< Integrated or fused yaw heading. */
    uint32_t timestamp_ms; /**< Timestamp of the latest IMU update. */
} GyroData;

/**
 * @brief Interpreted battery measurement and power safety state.
 */
typedef struct
{
    uint16_t raw_adc; /**< Averaged battery monitor ADC value. */
    float voltage_v; /**< Reconstructed battery-pack voltage. */
    bool low; /**< True when voltage is below the warning threshold. */
    bool critical; /**< True when voltage is below the motion-inhibit threshold. */
    uint32_t timestamp_ms; /**< Timestamp of the latest battery update. */
} BatteryStatus;

/**
 * @brief Coherent collection of the latest physical sensor measurements.
 */
typedef struct
{
    IRSensorData ir[IR_SENSOR_COUNT]; /**< IR measurements indexed by IRSensorId. */
    EncoderData left_encoder; /**< Left-wheel encoder measurement. */
    EncoderData right_encoder; /**< Right-wheel encoder measurement. */
    GyroData gyro; /**< IMU measurement and heading estimate. */
    BatteryStatus battery; /**< Battery measurement and status. */
    uint32_t timestamp_ms; /**< Timestamp at which the packet was assembled. */
} SensorPacket;

/* ========================================================================== */
/* Control and calibration types                                               */
/* ========================================================================== */

/**
 * @brief Runtime state and tuning parameters for a PID controller.
 */
typedef struct
{
    float kp; /**< Proportional gain. */
    float ki; /**< Integral gain. */
    float kd; /**< Derivative gain. */
    float setpoint; /**< Requested process value. */
    float measurement; /**< Latest measured process value. */
    float error; /**< Latest control error. */
    float previous_error; /**< Error from the preceding control cycle. */
    float integral; /**< Accumulated, clamped integral term. */
    float derivative; /**< Latest error-rate term. */
    float output; /**< Latest controller output. */
    float output_min; /**< Minimum permitted output. */
    float output_max; /**< Maximum permitted output. */
} PIDData;

/**
 * @brief Persistent calibration values collected from the assembled robot.
 */
typedef struct
{
    uint32_t magic; /**< Validation signature for stored calibration data. */
    uint32_t version; /**< Calibration data format version. */
    uint16_t ir_ambient_adc[IR_SENSOR_COUNT]; /**< Per-sensor ambient-light baselines. */
    uint16_t ir_wall_threshold_adc[IR_SENSOR_COUNT]; /**< Per-sensor wall thresholds. */
    float gyro_bias_dps[3]; /**< Stationary gyroscope bias for x, y, and z axes. */
    float left_wheel_scale; /**< Left encoder distance correction multiplier. */
    float right_wheel_scale; /**< Right encoder distance correction multiplier. */
    float battery_scale; /**< Battery conversion correction multiplier. */
    bool valid; /**< True after calibration data validation succeeds. */
} CalibrationData;

#ifdef __cplusplus
}
#endif

#endif /* TYPES_H */
