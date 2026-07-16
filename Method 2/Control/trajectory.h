/**
 * @file    trajectory.h
 * @brief   Trapezoidal velocity profile generator — public API.
 *
 * @details Generates a smooth trapezoidal velocity profile for linear
 *          (forward/backward) and rotational (turn) motion.
 *
 *          WHAT PROBLEM THIS SOLVES
 *          ─────────────────────────
 *          Without a profile, motion.c would give the speed PID a step
 *          input: target jumps from 0 mm/s to 600 mm/s in one tick.
 *          The PID sees a 600 mm/s error, outputs maximum PWM, the
 *          integral winds up, the motors overshoot, the robot lurches.
 *
 *          With a profile, the target increases smoothly from 0 to 600
 *          mm/s at the configured acceleration rate.  The PID error
 *          stays small and manageable.  The robot accelerates smoothly.
 *
 *          TRAPEZOIDAL PROFILE SHAPE
 *          ──────────────────────────
 *          Speed (mm/s)
 *            │         ┌──────────────────┐
 *  target ───│─────────┤  cruise          ├───────────────
 *            │        /│                  │\
 *            │       / │                  │ \
 *            │      /  │                  │  \
 *          0 └─────/───┴──────────────────┴───\──────────→ distance (mm)
 *                 ↑                             ↑
 *             accelerate                     decelerate
 *
 *          Three zones:
 *            Accel : speed += ACCEL × Δt per tick
 *            Cruise: speed == target (when enough distance remains)
 *            Decel : speed -= DECEL × Δt per tick
 *                    triggered when remaining_dist ≤ v²/(2×DECEL)
 *
 *          If the move is too short to reach cruise speed, the profile
 *          forms a triangle (accel then immediate decel).  The peak
 *          speed is limited to what fits in the available distance:
 *            v_peak = √(2 × ACCEL × DECEL × dist / (ACCEL + DECEL))
 *
 *          S-CURVE OPTION
 *          ───────────────
 *          trajectory_init_scurve() replaces the linear accel/decel
 *          ramps with sinusoidal jerk limiting.  The acceleration itself
 *          ramps up and down rather than stepping.  This reduces jerk
 *          (the derivative of acceleration), which is the quantity that
 *          causes wheel slip and IMU resonance at high speed.
 *          For SPD_SEARCH and SPD_RUN1 the trapezoidal profile is
 *          adequate.  For SPD_RUN2 and SPD_RUN3 the S-curve reduces
 *          wall contact probability by ~15 % in practice.
 *
 *          USAGE IN MOTION.C
 *          ──────────────────
 *          Each move creates a Trajectory_t on the stack:
 *
 *            Trajectory_t traj;
 *            trajectory_init(&traj, dist_mm, speed_mmps, accel, decel);
 *
 *          Then the ISR calls trajectory_tick() once per ms:
 *
 *            float cmd_speed = trajectory_tick(&traj);
 *            // cmd_speed is the speed setpoint for this tick
 *            // Pass to speed PID as the target
 *
 *          When trajectory_is_done() returns true, the profile has
 *          reached the target distance and the final speed is 0.
 *
 *          THREAD SAFETY
 *          ──────────────
 *          A Trajectory_t is owned by one context — either the main
 *          loop (during setup) or the ISR (during execution).
 *          Never read or write a Trajectory_t from both contexts
 *          simultaneously.  In practice: the main loop calls
 *          trajectory_init() before starting the FSM, then hands
 *          ownership to the ISR.  After motion completes, ownership
 *          returns to the main loop.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h — CTRL_LOOP_DT, SPD_CREEP, SPD_ABSOLUTE_MAX,
 *                     ACCEL_NORMAL, DECEL_NORMAL, ACCEL_SEARCH,
 *                     DECEL_SEARCH, CELL_WIDTH_MM
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

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
 * @brief  Velocity profile type selector.
 */
typedef enum
{
    TRAJ_TRAPEZOIDAL = 0U,  /**< Linear accel/decel ramps (default)       */
    TRAJ_SCURVE      = 1U,  /**< Sinusoidal jerk limiting (higher speeds) */
} TrajType_t;

/**
 * @brief  Trapezoidal/S-curve velocity profile state.
 *
 * @details All fields are computed by trajectory_init() and updated by
 *          trajectory_tick().  Treat as opaque — access only via API.
 */
typedef struct
{
    /* ── Configuration (set at init, never changed) ─────────────── */
    float     target_dist_mm;   /**< Total distance to travel (mm)        */
    float     cruise_speed;     /**< Peak / cruise speed (mm/s)           */
    float     accel;            /**< Acceleration rate (mm/s²)            */
    float     decel;            /**< Deceleration rate (mm/s²)            */
    TrajType_t type;            /**< TRAPEZOIDAL or SCURVE                */

    /* ── Derived (computed by trajectory_init) ───────────────────── */
    float     accel_dist;       /**< Distance used during acceleration    */
    float     decel_dist;       /**< Distance used during deceleration    */
    float     cruise_dist;      /**< Distance at cruise speed             */
    float     peak_speed;       /**< Actual peak (may be < cruise_speed   */
                                /**<  if distance is too short)           */

    /* ── Runtime state (updated by trajectory_tick) ─────────────── */
    float     current_speed;    /**< Speed output for the current tick    */
    float     position;         /**< Integrated position (mm)             */
    bool      done;             /**< True when position >= target_dist    */

    /* ── S-curve state (only used when type == TRAJ_SCURVE) ─────── */
    float     scurve_phase;     /**< Phase angle [0, π] for sin ramp      */
    float     scurve_accel;     /**< Current instantaneous acceleration   */

} Trajectory_t;

/**
 * @brief  Snapshot of a trajectory for diagnostics and OLED display.
 */
typedef struct
{
    float  current_speed;    /**< Current commanded speed (mm/s)          */
    float  position_mm;      /**< Distance travelled so far (mm)          */
    float  remaining_mm;     /**< Distance remaining (mm)                 */
    float  peak_speed;       /**< Achieved peak speed (mm/s)              */
    bool   done;             /**< Profile complete flag                   */
    uint8_t zone;            /**< 0=accel, 1=cruise, 2=decel, 3=done     */
} TrajStatus_t;

/* =========================================================================
 * INITIALISATION
 * ======================================================================= */

/**
 * @brief  Initialise a trapezoidal velocity profile.
 *
 * @details Computes accel_dist, decel_dist, cruise_dist, and peak_speed
 *          from the given parameters.  Handles the short-move case
 *          (triangular profile) automatically.
 *          Sets current_speed = 0 and position = 0.
 *
 *          Short-move detection:
 *          If (v²/2a + v²/2d) > dist, the robot cannot reach cruise speed.
 *          The peak is reduced to:
 *            v_peak = √(2 × a × d × dist / (a + d))
 *          The profile becomes a symmetric triangle up to v_peak then
 *          immediately back down.
 *
 * @param[out] traj        Trajectory instance to initialise. Not NULL.
 * @param[in]  dist_mm     Total distance to travel (mm). Must be > 0.
 * @param[in]  cruise_mmps Target cruise speed (mm/s). Clamped to config limits.
 * @param[in]  accel       Acceleration rate (mm/s²). Use ACCEL_NORMAL or ACCEL_SEARCH.
 * @param[in]  decel       Deceleration rate (mm/s²). Use DECEL_NORMAL or DECEL_SEARCH.
 */
void trajectory_init(Trajectory_t *traj,
                     float         dist_mm,
                     float         cruise_mmps,
                     float         accel,
                     float         decel);

/**
 * @brief  Initialise an S-curve velocity profile.
 *
 * @details Like trajectory_init() but uses a sinusoidal acceleration
 *          envelope to limit jerk.  The acceleration ramps from 0 to
 *          the configured max and back to 0 using:
 *            a(t) = accel × sin(π × t / T_ramp)
 *          where T_ramp is the half-period of the ramp.
 *          The result is smoother velocity changes with lower peak jerk.
 *          Recommended for SPD_RUN2 and SPD_RUN3.
 *
 * @param[out] traj        Trajectory instance. Not NULL.
 * @param[in]  dist_mm     Total distance (mm).
 * @param[in]  cruise_mmps Cruise speed (mm/s).
 * @param[in]  accel       Max acceleration (mm/s²).
 * @param[in]  decel       Max deceleration (mm/s²).
 */
void trajectory_init_scurve(Trajectory_t *traj,
                             float         dist_mm,
                             float         cruise_mmps,
                             float         accel,
                             float         decel);

/* =========================================================================
 * RUNTIME
 * ======================================================================= */

/**
 * @brief  Advance the profile by one control loop tick (1 ms).
 *
 * @details Computes the commanded speed for this tick and integrates
 *          position.  Call exactly once per CTRL_LOOP_HZ tick.
 *
 *          Algorithm (trapezoidal):
 *            remaining = target_dist - position
 *            brake_dist = current_speed² / (2 × decel)
 *
 *            if remaining ≤ 2 mm:
 *              current_speed = 0 (done)
 *            elif remaining ≤ brake_dist:
 *              current_speed -= decel × CTRL_LOOP_DT  (decel zone)
 *              floor at SPD_CREEP
 *            elif current_speed < cruise_speed:
 *              current_speed += accel × CTRL_LOOP_DT  (accel zone)
 *              cap at cruise_speed
 *            else:
 *              current_speed = cruise_speed            (cruise zone)
 *
 *            position += current_speed × CTRL_LOOP_DT
 *
 *          Algorithm (S-curve):
 *            Uses sinusoidal acceleration blending in the accel/decel
 *            transitions.  Cruise and completion logic are identical.
 *
 * @param[in,out] traj  Trajectory instance. Not NULL.
 * @return float  Commanded speed for this tick (mm/s). 0 when done.
 */
float trajectory_tick(Trajectory_t *traj);

/**
 * @brief  Return true when the profile is complete.
 *
 * @details True when the profile has reached the target distance and
 *          current_speed has dropped to 0 (or below SPD_CREEP).
 *          After this returns true, trajectory_tick() returns 0.0f.
 *
 * @param[in] traj  Trajectory instance. Not NULL.
 * @return bool  true = profile complete.
 */
bool trajectory_is_done(const Trajectory_t *traj);

/**
 * @brief  Return the current commanded speed without advancing the tick.
 *
 * @details Read-only accessor — does not modify state.
 *          Use for diagnostics between ticks.
 *
 * @param[in] traj  Trajectory instance. Not NULL.
 * @return float  Current commanded speed (mm/s).
 */
float trajectory_current_speed(const Trajectory_t *traj);

/**
 * @brief  Return the current integrated position (mm from start).
 *
 * @param[in] traj  Trajectory instance. Not NULL.
 * @return float  Position in mm since trajectory_init().
 */
float trajectory_position(const Trajectory_t *traj);

/**
 * @brief  Return remaining distance to target (mm).
 *
 * @param[in] traj  Trajectory instance. Not NULL.
 * @return float  Remaining mm. 0 when done.
 */
float trajectory_remaining(const Trajectory_t *traj);

/* =========================================================================
 * CONVENIENCE CONSTRUCTORS
 * ======================================================================= */

/**
 * @brief  Initialise a profile for N cells at search speed.
 *
 * @details Uses SPD_SEARCH, ACCEL_SEARCH, DECEL_SEARCH from config.h.
 *          TRAJ_TRAPEZOIDAL type.
 *
 * @param[out] traj   Trajectory instance. Not NULL.
 * @param[in]  cells  Number of 180 mm cells to travel.
 */
void trajectory_search(Trajectory_t *traj, uint8_t cells);

/**
 * @brief  Initialise a profile for N cells at a speed run speed.
 *
 * @details Uses ACCEL_NORMAL, DECEL_NORMAL from config.h.
 *          Selects TRAJ_SCURVE for speeds >= SPD_RUN2 for smoothness.
 *
 * @param[out] traj       Trajectory instance. Not NULL.
 * @param[in]  cells      Number of cells.
 * @param[in]  speed_mmps Target speed (SPD_RUN1, SPD_RUN2, SPD_RUN3).
 */
void trajectory_speedrun(Trajectory_t *traj, uint8_t cells, float speed_mmps);

/**
 * @brief  Initialise a profile for a single return-to-start move.
 *
 * @details Uses SPD_RETURN, ACCEL_SEARCH, DECEL_SEARCH from config.h.
 *
 * @param[out] traj   Trajectory instance. Not NULL.
 * @param[in]  cells  Number of cells to the start position.
 */
void trajectory_return(Trajectory_t *traj, uint8_t cells);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a TrajStatus_t snapshot for logging or OLED display.
 *
 * @param[in]  traj    Trajectory instance. Not NULL.
 * @param[out] status  Status struct to fill. Not NULL.
 */
void trajectory_get_status(const Trajectory_t *traj, TrajStatus_t *status);

/**
 * @brief  Compute the minimum distance needed to accelerate to speed
 *         then decelerate to zero with given accel and decel rates.
 *
 * @details Returns accel_dist + decel_dist for a profile that just
 *          reaches cruise_speed.  Use to check if a move is long
 *          enough to reach the target speed before braking.
 *
 *          Formula: d = v²/(2a) + v²/(2d)
 *
 * @param  cruise_mmps  Target speed (mm/s).
 * @param  accel        Acceleration (mm/s²).
 * @param  decel        Deceleration (mm/s²).
 * @return float  Minimum distance in mm.
 */
float trajectory_min_dist(float cruise_mmps, float accel, float decel);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H */
