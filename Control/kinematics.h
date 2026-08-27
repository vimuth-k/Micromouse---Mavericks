/**
 * @file    kinematics.h
 * @brief   Differential drive kinematics — public API.
 *
 * Provides conversions between wheel space (vL, vR) and robot space (v, omega),
 * encoder counts and physical distances/speeds, and turn geometry.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * TYPES
 * ----------------------------------------------------------------------- */

/** Robot-space velocity: linear + angular. */
typedef struct
{
    float v_mmps;    /**< Linear velocity of robot centre (mm/s)          */
    float omega_rps; /**< Angular velocity (rad/s), +ve = CW (right)      */
    float omega_dps; /**< Angular velocity (deg/s), derived from omega   */
} RobotVelocity_t;

/** Wheel-space velocity: left + right. */
typedef struct
{
    float vl_mmps;   /**< Left wheel speed (mm/s)                         */
    float vr_mmps;   /**< Right wheel speed (mm/s)                        */
} WheelVelocity_t;

/** Wheel-space PWM command: left + right. */
typedef struct
{
    int32_t pwm_left;    /**< Left motor PWM count [-PWM_MAX, PWM_MAX]   */
    int32_t pwm_right;   /**< Right motor PWM count [-PWM_MAX, PWM_MAX]  */
} WheelPWM_t;

/* -------------------------------------------------------------------------
 * ENCODER <-> PHYSICAL CONVERSIONS
 * ----------------------------------------------------------------------- */

/** Convert encoder counts to distance in mm (counts x MM_PER_COUNT). */
float kin_counts_to_mm(int32_t counts);

/** Convert distance in mm to nearest encoder count. */
int32_t kin_mm_to_counts(float dist_mm);

/** Convert encoder delta (counts per 1 ms tick) to speed in mm/s. */
float kin_delta_to_mmps(int32_t delta_counts);

/** Convert speed in mm/s to expected encoder delta per 1 ms tick. */
float kin_mmps_to_delta(float speed_mmps);

/** Convert cells to distance (cells x CELL_WIDTH_MM). */
float kin_cells_to_mm(uint8_t cells);

/** Convert distance (mm) to nearest cell count. */
uint8_t kin_mm_to_cells(float dist_mm);

/* -------------------------------------------------------------------------
 * WHEEL SPACE <-> ROBOT SPACE
 * ----------------------------------------------------------------------- */

/** Convert left and right wheel speeds to robot body velocity. */
void kin_wheels_to_robot(const WheelVelocity_t *wheels, RobotVelocity_t *robot);

/** Convert robot body velocity to left and right wheel speeds. */
void kin_robot_to_wheels(const RobotVelocity_t *robot, WheelVelocity_t *wheels);

/* -------------------------------------------------------------------------
 * SPEED <-> PWM (FEED-FORWARD)
 * ----------------------------------------------------------------------- */

/** Estimate open-loop PWM count for a given wheel speed (mm/s). */
int32_t kin_mmps_to_pwm(float speed_mmps);

/** Estimate wheel speed from a PWM count (open-loop). */
float kin_pwm_to_mmps(int32_t pwm);

/* -------------------------------------------------------------------------
 * TURN GEOMETRY
 * ----------------------------------------------------------------------- */

/** Compute wheel speeds for a pivot turn at a given angular rate (deg/s). */
void kin_pivot_wheel_speeds(float omega_dps, WheelVelocity_t *wheels);

/** Compute wheel speeds for an arc turn. */
void kin_arc_wheel_speeds(float v_mmps, float omega_dps, WheelVelocity_t *wheels);

/** Compute the turning radius for given robot speed and turn rate. */
float kin_turn_radius(float v_mmps, float omega_dps);

/** Compute the angular rate needed to complete a 90 deg pivot within time_s. */
float kin_turn_rate_for_time(float time_s);

/* -------------------------------------------------------------------------
 * ODOMETRY & ANGLES
 * ----------------------------------------------------------------------- */

/** Compute heading change (deg) from wheel distance delta (dist_R - dist_L). */
float kin_heading_change_deg(float dist_l_mm, float dist_r_mm);

/** Compute arc length each wheel travels during a pivot turn of angle_deg. */
float kin_pivot_arc_mm(float angle_deg);

/** Convert degrees to radians. */
float kin_deg_to_rad(float deg);

/** Convert radians to degrees. */
float kin_rad_to_deg(float rad);

/** Normalise an angle to (-180, +180] degrees. */
float kin_normalise_angle_deg(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
