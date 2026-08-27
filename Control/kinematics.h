/**
 * @file    kinematics.h
 * @brief   Differential drive kinematics — public API.
 *
 * @details Provides the mathematical conversions between the two
 *          representations of robot motion:
 *
 *          WHEEL SPACE     ←→    ROBOT SPACE
 *          (vL, vR mm/s)         (v mm/s, ω rad/s)
 *
 *          and between encoder counts and physical distances/speeds.
 *
 *          WHY THIS MODULE EXISTS
 *          ──────────────────────
 *          Different parts of the firmware think in different units:
 *
 *          motion.c     → thinks in mm/s per wheel (physical reality)
 *          pid.c        → thinks in PWM counts (actuator space)
 *          trajectory.c → thinks in mm/s (robot centre speed)
 *          maze.c       → thinks in cells (abstract navigation)
 *          encoders.c   → thinks in counts (sensor space)
 *          imu.c        → thinks in deg/s (sensor space)
 *
 *          kinematics.c is the translation layer that converts between
 *          these representations consistently, using the physical
 *          constants from config.h as the ground truth.
 *
 *          All conversions here are closed-form — no loops, no state,
 *          no hardware access.  Pure mathematics.
 *
 *          ROBOT MODEL
 *          ────────────
 *          Differential drive (two driven wheels on a shared axle).
 *
 *          Geometry (top-down view):
 *
 *                     ← B = WHEEL_SPACING_MM →
 *                     │                       │
 *                     ○ ← left wheel         ○ ← right wheel
 *                              ↑ robot forward
 *
 *          Kinematics equations:
 *
 *          Robot → Wheels:
 *            vL = v  −  ω × B/2
 *            vR = v  +  ω × B/2
 *
 *          Wheels → Robot:
 *            v  = (vL + vR) / 2
 *            ω  = (vR − vL) / B          [rad/s]
 *            ω° = ω × (180/π)            [deg/s]
 *
 *          Where:
 *            v  = linear  velocity of robot centre (mm/s)
 *            ω  = angular velocity of robot (rad/s, +CW from above)
 *            vL = left  wheel linear velocity (mm/s)
 *            vR = right wheel linear velocity (mm/s)
 *            B  = wheel spacing (mm) = WHEEL_SPACING_MM
 *
 *          ENCODER → PHYSICAL CONVERSION
 *          ───────────────────────────────
 *          Hardware:  7 PPR encoder on motor shaft, 30:1 gearbox,
 *                     quadrature (×4 edges) → 840 counts/wheel revolution.
 *          Wheel:     34 mm diameter → π×34 mm circumference.
 *          Scale:     0.12723 mm per encoder count (MM_PER_COUNT).
 *
 *          count → mm:   dist_mm  = count × MM_PER_COUNT
 *          count → mm/s: speed    = Δcount × MM_PER_COUNT × CTRL_LOOP_HZ
 *          mm → count:   count    = dist_mm / MM_PER_COUNT
 *          mm/s → PWM:   PWM ≈ speed × (PWM_MAX / SPD_ABSOLUTE_MAX)
 *                        (open-loop estimate — PID closes the loop)
 *
 *          TURNING GEOMETRY
 *          ─────────────────
 *          For a pivot turn (robot turns in place, centre of rotation
 *          = midpoint of axle):
 *            vL = −ω × B/2
 *            vR = +ω × B/2
 *            → vL = −vR (equal and opposite)
 *
 *          Arc turn (robot turns while moving forward):
 *            vL = v − ω × B/2
 *            vR = v + ω × B/2
 *            Turning radius: R = v / ω
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h  — WHEEL_SPACING_MM, MM_PER_COUNT, CTRL_LOOP_HZ,
 *                      COUNTS_PER_REV, PWM_MAX, SPD_ABSOLUTE_MAX,
 *                      WHEEL_DIAMETER_MM, WHEEL_CIRCUMFERENCE_MM
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

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  Robot-space velocity: linear + angular.
 */
typedef struct
{
    float v_mmps;    /**< Linear  velocity, robot centre (mm/s)          */
    float omega_rps; /**< Angular velocity (rad/s), +ve = CW from above  */
    float omega_dps; /**< Angular velocity (deg/s), derived from omega   */
} RobotVelocity_t;

/**
 * @brief  Wheel-space velocity: left + right.
 */
typedef struct
{
    float vl_mmps;   /**< Left  wheel speed (mm/s)                       */
    float vr_mmps;   /**< Right wheel speed (mm/s)                       */
} WheelVelocity_t;

/**
 * @brief  Wheel-space PWM command: left + right.
 */
typedef struct
{
    int32_t pwm_left;    /**< Left  motor PWM count [-PWM_MAX, PWM_MAX]  */
    int32_t pwm_right;   /**< Right motor PWM count [-PWM_MAX, PWM_MAX]  */
} WheelPWM_t;

/* =========================================================================
 * ENCODER ↔ PHYSICAL CONVERSIONS
 * ======================================================================= */

/**
 * @brief  Convert encoder counts to distance in mm.
 *
 * @details dist_mm = counts × MM_PER_COUNT
 *          MM_PER_COUNT = π × 34 / 840 = 0.12723 mm/count.
 *
 * @param  counts  Signed encoder count (positive = forward).
 * @return float   Distance in mm (positive = forward).
 */
float kin_counts_to_mm(int32_t counts);

/**
 * @brief  Convert distance in mm to encoder counts.
 *
 * @details counts = dist_mm / MM_PER_COUNT
 *          Rounds to nearest integer.
 *
 * @param  dist_mm  Distance in mm.
 * @return int32_t  Equivalent encoder counts.
 */
int32_t kin_mm_to_counts(float dist_mm);

/**
 * @brief  Convert encoder delta (counts per 1 ms tick) to speed in mm/s.
 *
 * @details speed_mmps = delta_counts × MM_PER_COUNT × CTRL_LOOP_HZ
 *          Valid only when called at exactly CTRL_LOOP_HZ (1 kHz).
 *
 * @param  delta_counts  Encoder counts since the last 1 ms tick.
 * @return float         Wheel speed in mm/s.
 */
float kin_delta_to_mmps(int32_t delta_counts);

/**
 * @brief  Convert speed in mm/s to expected encoder delta per 1 ms tick.
 *
 * @details delta = speed_mmps / (MM_PER_COUNT × CTRL_LOOP_HZ)
 *          Used to set encoder-based speed targets.
 *
 * @param  speed_mmps  Wheel speed in mm/s.
 * @return float       Expected encoder counts per tick (fractional).
 */
float kin_mmps_to_delta(float speed_mmps);

/* =========================================================================
 * WHEEL SPACE ↔ ROBOT SPACE
 * ======================================================================= */

/**
 * @brief  Convert left and right wheel speeds to robot body velocity.
 *
 * @details v     = (vL + vR) / 2
 *          omega = (vR − vL) / WHEEL_SPACING_MM    [rad/s]
 *
 * @param[in]  wheels  Wheel-space velocities (mm/s per wheel).
 * @param[out] robot   Robot-space velocity (v mm/s, ω rad/s, ω deg/s).
 */
void kin_wheels_to_robot(const WheelVelocity_t *wheels,
                               RobotVelocity_t  *robot);

/**
 * @brief  Convert robot body velocity to left and right wheel speeds.
 *
 * @details vL = v − ω × B/2
 *          vR = v + ω × B/2
 *          where B = WHEEL_SPACING_MM.
 *
 * @param[in]  robot   Robot-space velocity.
 * @param[out] wheels  Required left and right wheel speeds (mm/s).
 */
void kin_robot_to_wheels(const RobotVelocity_t *robot,
                               WheelVelocity_t  *wheels);

/* =========================================================================
 * SPEED ↔ PWM (OPEN-LOOP FEED-FORWARD)
 * ======================================================================= */

/**
 * @brief  Estimate open-loop PWM count for a given wheel speed.
 *
 * @details Provides a feed-forward term for the speed PID.  The PID
 *          closes the loop, but starting from a good open-loop estimate
 *          reduces the initial error and the integral wind-up time.
 *
 *          Linear approximation:
 *            PWM = speed_mmps × (PWM_MAX / SPD_ABSOLUTE_MAX)
 *            = speed_mmps × (4999 / 1335) ≈ speed_mmps × 3.74
 *
 *          The true motor characteristic is not perfectly linear
 *          (friction deadband at low speed, back-EMF at high speed),
 *          but this estimate is accurate to within ±10 % across the
 *          operating range.
 *
 * @param  speed_mmps  Desired wheel speed (mm/s). Negative = reverse.
 * @return int32_t     Estimated PWM count. Range: [-PWM_MAX, PWM_MAX].
 */
int32_t kin_mmps_to_pwm(float speed_mmps);

/**
 * @brief  Estimate wheel speed from a PWM count (open-loop).
 *
 * @details Inverse of kin_mmps_to_pwm():
 *            speed_mmps = pwm × (SPD_ABSOLUTE_MAX / PWM_MAX)
 *
 * @param  pwm  PWM count. Range: [-PWM_MAX, PWM_MAX].
 * @return float  Estimated wheel speed (mm/s).
 */
float kin_pwm_to_mmps(int32_t pwm);

/* =========================================================================
 * TURN GEOMETRY
 * ======================================================================= */

/**
 * @brief  Compute wheel speeds for a pivot turn at a given angular rate.
 *
 * @details Pivot turn: centre of rotation = midpoint of axle.
 *          vL = −omega × B/2
 *          vR = +omega × B/2
 *          Left and right are equal magnitude, opposite direction.
 *
 * @param  omega_dps   Target angular rate (deg/s). Positive = CW (right).
 * @param[out] wheels  Resulting left/right wheel speeds (mm/s).
 */
void kin_pivot_wheel_speeds(float omega_dps, WheelVelocity_t *wheels);

/**
 * @brief  Compute wheel speeds for an arc turn.
 *
 * @details Robot moves forward at v_mmps while turning at omega_dps.
 *          vL = v − ω × B/2      (ω in rad/s)
 *          vR = v + ω × B/2
 *          Inner wheel is slower, outer wheel is faster.
 *
 * @param  v_mmps      Forward speed of robot centre (mm/s).
 * @param  omega_dps   Turn rate (deg/s). Positive = CW.
 * @param[out] wheels  Required left/right wheel speeds (mm/s).
 */
void kin_arc_wheel_speeds(float v_mmps, float omega_dps,
                          WheelVelocity_t *wheels);

/**
 * @brief  Compute the turning radius for given robot speed and turn rate.
 *
 * @details R = v / ω   (ω in rad/s)
 *          R = 0 for pivot turns (omega → ∞ relative to v).
 *          Positive R = turning right (CW).
 *
 * @param  v_mmps     Forward speed (mm/s). Must be > 0.
 * @param  omega_dps  Angular rate (deg/s).
 * @return float      Turn radius in mm. Very large if nearly straight.
 */
float kin_turn_radius(float v_mmps, float omega_dps);

/**
 * @brief  Compute the angular rate needed to complete a 90° pivot
 *         within a target time.
 *
 * @details omega = 90° / time_s  [deg/s]
 *          Used to set a consistent turn speed regardless of speed profile.
 *
 * @param  time_s  Time to complete 90° turn (seconds).
 * @return float   Required angular rate (deg/s).
 */
float kin_turn_rate_for_time(float time_s);

/* =========================================================================
 * ODOMETRY
 * ======================================================================= */

/**
 * @brief  Compute robot heading change from left/right wheel distances.
 *
 * @details Δθ = (dist_R − dist_L) / WHEEL_SPACING_MM   [rad]
 *          Positive = clockwise (turning right).
 *          Used to cross-check gyro readings with encoder-based odometry.
 *
 * @param  dist_l_mm  Left  wheel distance this interval (mm).
 * @param  dist_r_mm  Right wheel distance this interval (mm).
 * @return float      Heading change in degrees (positive = CW).
 */
float kin_heading_change_deg(float dist_l_mm, float dist_r_mm);

/**
 * @brief  Compute the arc length each wheel travels during a pure pivot
 *         turn of a given angle.
 *
 * @details Each wheel travels along a circle of radius B/2:
 *          arc = (B/2) × |θ_rad|
 *          Used to set encoder-count targets for gyro-free turns.
 *
 * @param  angle_deg  Turn angle (degrees, magnitude only).
 * @return float      Arc length per wheel (mm).
 */
float kin_pivot_arc_mm(float angle_deg);

/* =========================================================================
 * UNIT CONVERSIONS
 * ======================================================================= */

/**
 * @brief  Convert degrees to radians.
 * @param  deg  Angle in degrees.
 * @return float  Angle in radians.
 */
float kin_deg_to_rad(float deg);

/**
 * @brief  Convert radians to degrees.
 * @param  rad  Angle in radians.
 * @return float  Angle in degrees.
 */
float kin_rad_to_deg(float rad);

/**
 * @brief  Normalise an angle to the range (-180°, +180°].
 *
 * @details Wraps any angle into the principal range.
 *          Used to compute the shortest angular error for turn PID.
 *          Example: 270° → −90°.  Example: −200° → 160°.
 *
 * @param  angle_deg  Input angle in degrees (any value).
 * @return float      Equivalent angle in (-180°, +180°].
 */
float kin_normalise_angle_deg(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
