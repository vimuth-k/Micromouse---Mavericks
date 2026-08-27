/**
 * @file    kinematics.c
 * @brief   Differential drive kinematics — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          Every function in this file is a pure mathematical conversion
 *          with no state, no side effects, and no hardware access.
 *          Safe to call from any context including the 1 kHz ISR.
 *
 *          Five conversion groups:
 *
 *            1. ENCODER ↔ DISTANCE
 *               The fundamental unit of the drivetrain.
 *               MM_PER_COUNT = π × 34 / 840 = 0.12723 mm/count.
 *               Every distance in the firmware comes from multiplying
 *               an encoder count by this constant.
 *
 *            2. SPEED ↔ PWM  (open-loop feed-forward)
 *               Maps desired speed to a starting PWM for the speed PID.
 *               Only needs to be approximately correct — PID fixes error.
 *               Reduces the PID transient at the start of each move.
 *
 *            3. WHEEL SPEEDS ↔ ROBOT VELOCITY
 *               Standard differential drive kinematic equations.
 *               Forward:  (vL, vR) → (v_linear, v_angular)
 *               Inverse:  (v_linear, v_angular) → (vL, vR)
 *
 *            4. TURN GEOMETRY
 *               Derives wheel speeds for curved paths and pivot turns.
 *               Also converts heading angles ↔ wheel arc distances —
 *               used to cross-check gyro with encoder odometry.
 *
 *            5. CELL AND UNIT CONVERSIONS
 *               Maze cells ↔ mm, degrees ↔ radians.
 *
 *          WHY SEPARATE FROM MOTION.C
 *          ───────────────────────────
 *          motion.c owns PID state and the FSM. kinematics.c is pure math.
 *          turn.c, diagnostics.c, and explorer.c all need these conversions
 *          — centralising them means one fix propagates everywhere.
 *          These functions are also trivially testable on a PC without any
 *          hardware, which catches formula bugs before flashing.
 *
 * @author  VDawn
 * @date    2026
 */

#include "kinematics.h"
#include "config.h"
#include <math.h>

/* =========================================================================
 * PRIVATE CONSTANTS  (derived from config.h, computed once)
 * ======================================================================= */

#define KIN_PI          3.14159265f

/** Feed-forward PWM per mm/s: PWM_MAX / SPD_ABSOLUTE_MAX = 4999/1335 = 3.745 */
static const float FF_PWM_PER_MMPS =
    (float)PWM_MAX / SPD_ABSOLUTE_MAX;

/** mm/s per encoder count per tick: MM_PER_COUNT × CTRL_LOOP_HZ */
static const float MMPS_PER_COUNT_PER_TICK =
    MM_PER_COUNT * (float)CTRL_LOOP_HZ;

/** Half wheel spacing in mm */
static const float HALF_TRACK = WHEEL_SPACING_MM * 0.5f;

/* =========================================================================
 * PRIVATE HELPER
 * ======================================================================= */

static float clamp_f(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/* =========================================================================
 * ENCODER <-> DISTANCE
 * ======================================================================= */

/**
 * @brief  Convert encoder counts to distance (mm).
 *
 * @details dist = counts × MM_PER_COUNT
 *          MM_PER_COUNT = π × 34 / 840 = 0.12723 mm/count.
 */
float kin_counts_to_mm(int32_t counts)
{
    return (float)counts * MM_PER_COUNT;
}

/**
 * @brief  Convert distance (mm) to nearest encoder count.
 *
 * @details counts = round(dist / MM_PER_COUNT)
 */
int32_t kin_mm_to_counts(float dist_mm)
{
    float raw = dist_mm / MM_PER_COUNT;
    if (raw >= 0.0f) { return (int32_t)(raw + 0.5f); }
    return              (int32_t)(raw - 0.5f);
}

/**
 * @brief  Convert encoder delta (counts per 1 ms tick) to speed (mm/s).
 *
 * @details speed = delta × MM_PER_COUNT × CTRL_LOOP_HZ
 *          = delta × 127.23  mm/s per count/tick.
 *
 *          At 200 mm/s: delta ≈ 1.57 counts/tick.
 *          At 750 mm/s: delta ≈ 5.90 counts/tick.
 */
float kin_delta_to_mmps(int32_t delta_counts)
{
    return (float)delta_counts * MMPS_PER_COUNT_PER_TICK;
}

/**
 * @brief  Convert speed (mm/s) to expected encoder delta per tick.
 *
 * @details delta = speed / MMPS_PER_COUNT_PER_TICK
 *          Inverse of kin_delta_to_mmps().
 */
float kin_mmps_to_delta(float speed_mmps)
{
    if (MMPS_PER_COUNT_PER_TICK <= 0.0f) { return 0.0f; }
    return speed_mmps / MMPS_PER_COUNT_PER_TICK;
}

/* =========================================================================
 * SPEED <-> PWM
 * ======================================================================= */

/**
 * @brief  Convert desired speed (mm/s) to open-loop PWM feed-forward.
 *
 * @details pwm = speed × (PWM_MAX / SPD_ABSOLUTE_MAX)
 *          = speed × 3.745
 *          Clamped to [0, PWM_MAX].
 *
 *          Not accurate under load — the speed PID corrects error.
 *          Feed-forward gives the PID a good starting point.
 */
float kin_mmps_to_pwm_ff(float speed_mmps)
{
    return clamp_f(speed_mmps * FF_PWM_PER_MMPS, 0.0f, (float)PWM_MAX);
}

/**
 * @brief  Convert PWM count to estimated wheel speed (mm/s).
 *
 * @details speed = pwm / FF_PWM_PER_MMPS. Open-loop estimate only.
 */
float kin_pwm_to_mmps(float pwm)
{
    if (FF_PWM_PER_MMPS <= 0.0f) { return 0.0f; }
    return clamp_f(pwm, 0.0f, (float)PWM_MAX) / FF_PWM_PER_MMPS;
}

/* =========================================================================
 * DIFFERENTIAL DRIVE: WHEEL SPEEDS <-> ROBOT VELOCITY
 * ======================================================================= */

/**
 * @brief  Forward kinematics: (vL, vR) → (v_linear, v_angular).
 *
 * @details v  = (vL + vR) / 2          linear velocity (mm/s)
 *          ω  = (vR − vL) / B          angular velocity (rad/s)
 *          where B = WHEEL_SPACING_MM = 82 mm.
 *
 *          Sign: ω > 0 = counter-clockwise (left turn).
 *
 *          Examples:
 *            vL=200, vR=200 → v=200,  ω=0       (straight)
 *            vL=0,   vR=200 → v=100,  ω=2.44    (left arc)
 *            vL=-200,vR=200 → v=0,    ω=4.88    (pivot left)
 */
void kin_wheels_to_robot(float  vl, float  vr,
                          float *v_linear, float *v_angular)
{
    if (v_linear  != NULL) { *v_linear  = (vl + vr) * 0.5f; }
    if (v_angular != NULL) { *v_angular = (vr - vl) / WHEEL_SPACING_MM; }
}

/**
 * @brief  Inverse kinematics: (v_linear, v_angular) → (vL, vR).
 *
 * @details vL = v − ω × B/2
 *          vR = v + ω × B/2
 *
 *          Examples:
 *            v=200, ω=0   → vL=200, vR=200   (straight)
 *            v=200, ω=1.0 → vL=159, vR=241   (gentle left arc)
 *            v=0,   ω=4.88→ vL=-200,vR=200   (pivot left)
 */
void kin_robot_to_wheels(float  v_linear, float  v_angular,
                          float *vl,       float *vr)
{
    float half_omega_B = v_angular * HALF_TRACK;
    if (vl != NULL) { *vl = v_linear - half_omega_B; }
    if (vr != NULL) { *vr = v_linear + half_omega_B; }
}

/* =========================================================================
 * TURN GEOMETRY
 * ======================================================================= */

/**
 * @brief  Compute wheel speeds for a turn with given radius and speed.
 *
 * @details For radius R and forward speed v at robot centre:
 *            ω  = v / R
 *            vL = v − ω × B/2 = v × (1 − B/(2R))
 *            vR = v + ω × B/2 = v × (1 + B/(2R))
 *
 *          Special case R=0 (pivot):
 *            vL = −v,  vR = +v
 *
 *          Radius sign: R>0 = left turn (CCW), R<0 = right turn (CW).
 *
 *          Example: v=300, R=180mm (left turn, one-cell radius)
 *            ω  = 300/180 = 1.667 rad/s
 *            vL = 300×(1 − 82/360) = 231.7 mm/s
 *            vR = 300×(1 + 82/360) = 368.3 mm/s
 */
void kin_turn_radius_to_wheels(float  forward_speed_mmps,
                                float  turn_radius_mm,
                                float *vl, float *vr)
{
    if (turn_radius_mm == 0.0f)
    {
        /* Pivot turn: opposite equal speeds */
        if (vl != NULL) { *vl = -forward_speed_mmps; }
        if (vr != NULL) { *vr =  forward_speed_mmps; }
        return;
    }

    float omega        = forward_speed_mmps / turn_radius_mm;
    float half_omega_B = omega * HALF_TRACK;
    if (vl != NULL) { *vl = forward_speed_mmps - half_omega_B; }
    if (vr != NULL) { *vr = forward_speed_mmps + half_omega_B; }
}

/**
 * @brief  Compute heading change (deg) from encoder wheel distance difference.
 *
 * @details Δθ_rad = (dist_right − dist_left) / WHEEL_SPACING_MM
 *          Δθ_deg = Δθ_rad × (180 / π)
 *          Positive = clockwise (right turn).
 *
 *          Use for odometric cross-checks against gyro — not as a primary
 *          heading source (accumulates drift from slip and wheel diameter
 *          measurement error).
 *
 *          Example: left=0mm, right=64.5mm (90° pivot right)
 *            Δθ = (64.5−0)/82 × (180/π) = 45.1° ← half turn only;
 *            pivot: left goes backward too, so left=−64.5mm:
 *            Δθ = (64.5−(−64.5))/82 × (180/π) = 90.0° ✓
 */
float kin_heading_change_deg(float dist_left_mm, float dist_right_mm)
{
    float delta_rad = (dist_right_mm - dist_left_mm) / WHEEL_SPACING_MM;
    return delta_rad * (180.0f / KIN_PI);
}

/**
 * @brief  Compute arc distance each wheel travels for a pivot turn.
 *
 * @details For a pivot of angle_deg degrees (robot centre stationary):
 *            arc = |angle_deg| × (π/180) × B/2
 *          where B/2 = HALF_TRACK = 41 mm.
 *          Left wheel goes backward this distance,
 *          right wheel goes forward this distance (for a right turn).
 *
 *          Example: 90° turn
 *            arc = 90 × 0.01745 × 41 = 64.5 mm
 *
 *          Example: 180° U-turn
 *            arc = 180 × 0.01745 × 41 = 128.8 mm
 */
float kin_turn_arc_mm(float angle_deg)
{
    float angle_rad = fabsf(angle_deg) * (KIN_PI / 180.0f);
    return angle_rad * HALF_TRACK;
}

/* =========================================================================
 * CELL GEOMETRY
 * ======================================================================= */

/**
 * @brief  Convert cells to distance (mm).
 *
 * @details dist = cells × CELL_WIDTH_MM = cells × 180 mm.
 */
float kin_cells_to_mm(uint8_t cells)
{
    return (float)cells * CELL_WIDTH_MM;
}

/**
 * @brief  Convert distance (mm) to nearest cell count.
 *
 * @details cells = round(dist / CELL_WIDTH_MM). Clamped to [0, 255].
 */
uint8_t kin_mm_to_cells(float dist_mm)
{
    if (dist_mm <= 0.0f) { return 0U; }
    uint32_t c = (uint32_t)((dist_mm / CELL_WIDTH_MM) + 0.5f);
    return (c > 255U) ? 255U : (uint8_t)c;
}

/* =========================================================================
 * UNIT CONVERSIONS
 * ======================================================================= */

/**
 * @brief  Degrees to radians.
 */
float kin_deg_to_rad(float deg)
{
    return deg * (KIN_PI / 180.0f);
}

/**
 * @brief  Radians to degrees.
 */
float kin_rad_to_deg(float rad)
{
    return rad * (180.0f / KIN_PI);
}
