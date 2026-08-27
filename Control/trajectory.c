/**
 * @file    trajectory.c
 * @brief   Trapezoidal and S-curve velocity profile generator — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *
 *          trajectory.c generates the speed setpoint that the speed PID
 *          in motion.c tracks each millisecond.  It is the only file
 *          that does this — motion.c calls trajectory_tick() to get the
 *          commanded speed for the current tick, then passes that as the
 *          PID setpoint.
 *
 *          It does NOT:
 *            - control motors directly
 *            - read encoders or sensors
 *            - know anything about walls, cells, or the maze
 *          It only answers one question per tick: "what speed should the
 *          robot be travelling at right now?"
 *
 *          TWO PROFILE TYPES
 *          ──────────────────
 *
 *          A. TRAPEZOIDAL  (TRAJ_TRAPEZOIDAL)
 *          ─────────────────────────────────
 *          The classical embedded motion profile.  Three zones:
 *
 *            Zone 1 (Acceleration):
 *              speed += ACCEL × Δt each tick
 *              Until speed reaches cruise_speed
 *              OR until remaining distance ≤ brake_dist (triangle case)
 *
 *            Zone 2 (Cruise):
 *              speed = cruise_speed
 *              Until remaining ≤ brake_dist
 *
 *            Zone 3 (Deceleration):
 *              triggered when: remaining ≤ current_speed² / (2 × DECEL)
 *              speed -= DECEL × Δt each tick
 *              Floor: SPD_CREEP (prevents stall)
 *              Until: remaining ≤ 2 mm → speed = 0, done = true
 *
 *          Braking distance check runs EVERY tick, including during
 *          acceleration — this handles the short-move (triangle) case:
 *          if the move is too short to reach cruise speed, the decel
 *          condition triggers while still accelerating and the profile
 *          smoothly peaks then descends.
 *
 *          B. S-CURVE  (TRAJ_SCURVE)
 *          ─────────────────────────
 *          Extends the trapezoidal profile with sinusoidal jerk limiting.
 *          Instead of stepping the acceleration from 0 to ACCEL instantly
 *          (infinite jerk), the acceleration itself ramps:
 *
 *            a(t) = accel_max × sin(π × t / T_half)
 *
 *          where T_half is the half-period that brings speed from 0 to
 *          cruise_speed with the same total impulse as the trapezoidal ramp.
 *
 *          Effect on robot: wheel slip and floor bump resonance are
 *          reduced at the start of acceleration.  The IMU sees a smoother
 *          angular acceleration signal, reducing gyro noise spikes.
 *          The total acceleration time is ≈ π/2 × (trapezoidal time),
 *          so the S-curve takes slightly longer to reach cruise speed.
 *
 *          At SPD_SEARCH (200 mm/s) the benefit is marginal — use
 *          trapezoidal.  At SPD_RUN3 (750 mm/s) the S-curve measurably
 *          reduces wall contact during the acceleration phase.
 *
 *          SHORT MOVE HANDLING
 *          ────────────────────
 *          If dist < v²/(2a) + v²/(2d), the robot cannot reach cruise speed.
 *          trajectory_init() detects this and computes a reduced peak:
 *
 *            v_peak = √(2 × a × d × dist / (a + d))
 *
 *          Derivation: solve for v where (v²/2a) + (v²/2d) = dist
 *            v² × (1/2a + 1/2d) = dist
 *            v² × (a+d)/(2ad)   = dist
 *            v² = 2ad × dist / (a+d)
 *            v  = √(2ad × dist / (a+d))
 *
 *          The profile then forms a triangle up to v_peak.
 *          This occurs for single-cell moves (180 mm) at SPD_RUN3
 *          (750 mm/s): brake_dist alone = 750²/(2×4000) = 70.3 mm,
 *          accel_dist = 750²/(2×3000) = 93.75 mm.  Total = 164 mm.
 *          Cruise distance = 180 - 164 = 16 mm — very short cruise.
 *          For multi-cell moves this is not a concern.
 *
 * @author  VDawn
 * @date    2026
 */

#include "trajectory.h"
#include "config.h"
#include <math.h>
#include <string.h>

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Clamp a float to [lo, hi].
 */
static float clamp_f(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/**
 * @brief  Compute the trapezoidal profile geometry at init time.
 *
 * @details Determines accel_dist, decel_dist, cruise_dist, and peak_speed.
 *          Handles the short-move (triangle) case.
 *
 * @param[in,out] traj  Trajectory instance with target_dist, cruise_speed,
 *                      accel, and decel already set.
 */
static void compute_trapezoid_geometry(Trajectory_t *traj)
{
    /* Distance to accelerate from 0 to cruise, and decelerate from cruise to 0 */
    float a_dist = (traj->cruise_speed * traj->cruise_speed) / (2.0f * traj->accel);
    float d_dist = (traj->cruise_speed * traj->cruise_speed) / (2.0f * traj->decel);
    float full_dist = a_dist + d_dist;

    if (full_dist >= traj->target_dist_mm)
    {
        /* Short move — cannot reach cruise speed.  Form a triangle.    */
        /* v_peak = √(2 × a × d × dist / (a + d))                      */
        float v_peak = sqrtf(2.0f * traj->accel * traj->decel
                             * traj->target_dist_mm
                             / (traj->accel + traj->decel));

        traj->peak_speed  = clamp_f(v_peak, SPD_CREEP, traj->cruise_speed);
        traj->accel_dist  = (traj->peak_speed * traj->peak_speed) / (2.0f * traj->accel);
        traj->decel_dist  = traj->target_dist_mm - traj->accel_dist;
        traj->cruise_dist = 0.0f;
    }
    else
    {
        /* Normal trapezoidal — cruise zone exists */
        traj->peak_speed  = traj->cruise_speed;
        traj->accel_dist  = a_dist;
        traj->decel_dist  = d_dist;
        traj->cruise_dist = traj->target_dist_mm - a_dist - d_dist;
    }
}

/* =========================================================================
 * PUBLIC API — INITIALISATION
 * ======================================================================= */

/**
 * @brief  Initialise a trapezoidal velocity profile.
 */
void trajectory_init(Trajectory_t *traj,
                     float         dist_mm,
                     float         cruise_mmps,
                     float         accel,
                     float         decel)
{
    if (traj == NULL) { return; }

    (void)memset(traj, 0, sizeof(Trajectory_t));

    traj->target_dist_mm = (dist_mm > 0.0f) ? dist_mm : 0.0f;
    traj->cruise_speed   = clamp_f(cruise_mmps, SPD_CREEP, SPD_ABSOLUTE_MAX);
    traj->accel          = (accel > 0.0f) ? accel : ACCEL_NORMAL;
    traj->decel          = (decel > 0.0f) ? decel : DECEL_NORMAL;
    traj->type           = TRAJ_TRAPEZOIDAL;
    traj->current_speed  = 0.0f;
    traj->position       = 0.0f;
    traj->done           = (dist_mm <= 0.0f);

    compute_trapezoid_geometry(traj);
}

/**
 * @brief  Initialise an S-curve velocity profile.
 *
 * @details The S-curve uses the same geometry as the trapezoid for
 *          distance planning, but replaces the constant acceleration
 *          with a sinusoidal envelope during the ramp phases.
 *          The scurve_phase variable tracks position within the sine
 *          half-cycle: 0 → π during acceleration, 0 → π during decel.
 */
void trajectory_init_scurve(Trajectory_t *traj,
                             float         dist_mm,
                             float         cruise_mmps,
                             float         accel,
                             float         decel)
{
    /* Start with a standard trapezoid geometry */
    trajectory_init(traj, dist_mm, cruise_mmps, accel, decel);

    /* Override type — tick function will handle sinusoidal ramps */
    traj->type        = TRAJ_SCURVE;
    traj->scurve_phase = 0.0f;
    traj->scurve_accel = 0.0f;
}

/* =========================================================================
 * PUBLIC API — RUNTIME
 * ======================================================================= */

/**
 * @brief  Advance the trapezoidal profile by one tick.
 *
 * @details Zone determination order:
 *          1. Check done (remaining ≤ 2 mm) first.
 *          2. Check brake condition (remaining ≤ brake_dist).
 *          3. Check if still accelerating.
 *          4. Otherwise cruise.
 *
 *          The brake_dist check runs every tick regardless of zone.
 *          This ensures correct behaviour in the triangle case where
 *          the brake condition can trigger during acceleration.
 *
 * @param[in,out] traj  Trajectory instance.
 * @return float  Commanded speed for this tick (mm/s).
 */
static float tick_trapezoid(Trajectory_t *traj)
{
    float remaining = traj->target_dist_mm - traj->position;

    /* ── ZONE: done ─────────────────────────────────────────────── */
    if (remaining <= 2.0f)
    {
        traj->current_speed = 0.0f;
        traj->done          = true;
        return 0.0f;
    }

    /* ── Braking distance for current speed ────────────────────── */
    float brake_dist = (traj->current_speed * traj->current_speed)
                       / (2.0f * traj->decel);

    /* ── ZONE: decelerate ───────────────────────────────────────── */
    if (remaining <= brake_dist)
    {
        float new_spd = traj->current_speed - (traj->decel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, SPD_CREEP, traj->current_speed);
    }
    /* ── ZONE: accelerate ───────────────────────────────────────── */
    else if (traj->current_speed < traj->peak_speed)
    {
        float new_spd = traj->current_speed + (traj->accel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, 0.0f, traj->peak_speed);
    }
    /* ── ZONE: cruise ───────────────────────────────────────────── */
    else
    {
        traj->current_speed = traj->peak_speed;
    }

    /* Integrate position */
    traj->position += traj->current_speed * CTRL_LOOP_DT;

    return traj->current_speed;
}

/**
 * @brief  Advance the S-curve profile by one tick.
 *
 * @details Uses the same zone logic as the trapezoid for cruise and
 *          completion detection.  Replaces the linear accel/decel ramps
 *          with sinusoidal acceleration envelopes.
 *
 *          During acceleration:
 *            phase increments by Δphase = π × accel × Δt / cruise_speed
 *            (derived from: total phase π corresponds to full 0→v ramp)
 *            instantaneous accel = accel_max × sin(phase)
 *            speed += instantaneous_accel × Δt
 *
 *          During deceleration:
 *            same formula but speed -= instantaneous_accel × Δt
 *
 *          The sine envelope means acceleration starts at 0, peaks at
 *          accel_max at the halfway point, and returns to 0 at the end
 *          of the ramp.  This gives zero jerk at the transition points.
 */
static float tick_scurve(Trajectory_t *traj)
{
    float remaining = traj->target_dist_mm - traj->position;

    /* ── ZONE: done ─────────────────────────────────────────────── */
    if (remaining <= 2.0f)
    {
        traj->current_speed = 0.0f;
        traj->done          = true;
        return 0.0f;
    }

    float brake_dist = (traj->current_speed * traj->current_speed)
                       / (2.0f * traj->decel);

    /* ── ZONE: decelerate (S-curve) ─────────────────────────────── */
    if (remaining <= brake_dist)
    {
        /* Phase increments such that sin integral matches linear decel */
        /* Δphase = π × decel_max × Δt / peak_speed                   */
        float d_phase = (float)M_PI * traj->decel * CTRL_LOOP_DT
                        / traj->peak_speed;
        traj->scurve_phase += d_phase;
        if (traj->scurve_phase > (float)M_PI)
        {
            traj->scurve_phase = (float)M_PI;
        }

        float inst_decel = traj->decel * sinf(traj->scurve_phase);
        float new_spd = traj->current_speed - (inst_decel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, SPD_CREEP, traj->current_speed);
    }
    /* ── ZONE: accelerate (S-curve) ─────────────────────────────── */
    else if (traj->current_speed < traj->peak_speed)
    {
        float d_phase = (float)M_PI * traj->accel * CTRL_LOOP_DT
                        / traj->peak_speed;
        traj->scurve_phase += d_phase;
        if (traj->scurve_phase > (float)M_PI)
        {
            traj->scurve_phase = (float)M_PI;
        }

        float inst_accel = traj->accel * sinf(traj->scurve_phase);
        float new_spd = traj->current_speed + (inst_accel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, 0.0f, traj->peak_speed);

        /* Reset phase when we enter cruise (ready for decel ramp) */
        if (traj->current_speed >= traj->peak_speed)
        {
            traj->scurve_phase = 0.0f;
        }
    }
    /* ── ZONE: cruise ───────────────────────────────────────────── */
    else
    {
        traj->current_speed = traj->peak_speed;
        traj->scurve_phase  = 0.0f;   /* Reset for upcoming decel ramp */
    }

    traj->position += traj->current_speed * CTRL_LOOP_DT;
    return traj->current_speed;
}

/**
 * @brief  Advance the profile by one control loop tick.
 *
 * @details Dispatches to tick_trapezoid() or tick_scurve() based on type.
 */
float trajectory_tick(Trajectory_t *traj)
{
    if (traj == NULL || traj->done) { return 0.0f; }

    if (traj->type == TRAJ_SCURVE)
    {
        return tick_scurve(traj);
    }
    return tick_trapezoid(traj);
}

/* =========================================================================
 * PUBLIC API — READ-ONLY ACCESSORS
 * ======================================================================= */

bool  trajectory_is_done(const Trajectory_t *traj)
{
    return (traj != NULL) ? traj->done : true;
}

float trajectory_current_speed(const Trajectory_t *traj)
{
    return (traj != NULL) ? traj->current_speed : 0.0f;
}

float trajectory_position(const Trajectory_t *traj)
{
    return (traj != NULL) ? traj->position : 0.0f;
}

float trajectory_remaining(const Trajectory_t *traj)
{
    if (traj == NULL) { return 0.0f; }
    float rem = traj->target_dist_mm - traj->position;
    return (rem > 0.0f) ? rem : 0.0f;
}

/* =========================================================================
 * PUBLIC API — CONVENIENCE CONSTRUCTORS
 * ======================================================================= */

/**
 * @brief  Search-speed profile for N cells.
 */
void trajectory_search(Trajectory_t *traj, uint8_t cells)
{
    trajectory_init(traj,
                    (float)cells * CELL_WIDTH_MM,
                    SPD_SEARCH,
                    ACCEL_SEARCH,
                    DECEL_SEARCH);
}

/**
 * @brief  Speed-run profile for N cells at a given speed.
 *
 * @details Uses S-curve for SPD_RUN2 and above, trapezoidal for slower.
 */
void trajectory_speedrun(Trajectory_t *traj, uint8_t cells, float speed_mmps)
{
    float dist = (float)cells * CELL_WIDTH_MM;

    if (speed_mmps >= SPD_RUN2)
    {
        trajectory_init_scurve(traj, dist, speed_mmps, ACCEL_NORMAL, DECEL_NORMAL);
    }
    else
    {
        trajectory_init(traj, dist, speed_mmps, ACCEL_NORMAL, DECEL_NORMAL);
    }
}

/**
 * @brief  Return-to-start profile for N cells.
 */
void trajectory_return(Trajectory_t *traj, uint8_t cells)
{
    trajectory_init(traj,
                    (float)cells * CELL_WIDTH_MM,
                    SPD_RETURN,
                    ACCEL_SEARCH,
                    DECEL_SEARCH);
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Determine which zone the profile is currently in.
 *
 * @return uint8_t  0=accel, 1=cruise, 2=decel, 3=done.
 */
static uint8_t current_zone(const Trajectory_t *traj)
{
    if (traj->done)                              { return 3U; }
    float brake = (traj->current_speed * traj->current_speed)
                  / (2.0f * traj->decel);
    float rem   = traj->target_dist_mm - traj->position;
    if (rem <= 2.0f)                             { return 3U; }
    if (rem <= brake)                            { return 2U; }
    if (traj->current_speed < traj->peak_speed) { return 0U; }
    return 1U;
}

/**
 * @brief  Populate a TrajStatus_t snapshot.
 */
void trajectory_get_status(const Trajectory_t *traj, TrajStatus_t *status)
{
    if (traj == NULL || status == NULL) { return; }

    status->current_speed = traj->current_speed;
    status->position_mm   = traj->position;
    status->remaining_mm  = trajectory_remaining(traj);
    status->peak_speed    = traj->peak_speed;
    status->done          = traj->done;
    status->zone          = current_zone(traj);
}

/**
 * @brief  Compute the minimum distance needed for a full accel + decel.
 */
float trajectory_min_dist(float cruise_mmps, float accel, float decel)
{
    if (accel <= 0.0f || decel <= 0.0f) { return 0.0f; }
    float a_dist = (cruise_mmps * cruise_mmps) / (2.0f * accel);
    float d_dist = (cruise_mmps * cruise_mmps) / (2.0f * decel);
    return a_dist + d_dist;
}
