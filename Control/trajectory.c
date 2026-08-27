/**
 * @file    trajectory.c
 * @brief   Trapezoidal and S-curve velocity profile generator — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "trajectory.h"
#include "config.h"
#include <math.h>
#include <string.h>

static float clamp_f(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

static void compute_trapezoid_geometry(Trajectory_t *traj)
{
    float a_dist = (traj->cruise_speed * traj->cruise_speed) / (2.0f * traj->accel);
    float d_dist = (traj->cruise_speed * traj->cruise_speed) / (2.0f * traj->decel);
    float full_dist = a_dist + d_dist;

    if (full_dist >= traj->target_dist_mm)
    {
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
        traj->peak_speed  = traj->cruise_speed;
        traj->accel_dist  = a_dist;
        traj->decel_dist  = d_dist;
        traj->cruise_dist = traj->target_dist_mm - a_dist - d_dist;
    }
}

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

void trajectory_init_scurve(Trajectory_t *traj,
                             float         dist_mm,
                             float         cruise_mmps,
                             float         accel,
                             float         decel)
{
    trajectory_init(traj, dist_mm, cruise_mmps, accel, decel);

    traj->type        = TRAJ_SCURVE;
    traj->scurve_phase = 0.0f;
    traj->scurve_accel = 0.0f;
}

static float tick_trapezoid(Trajectory_t *traj)
{
    float remaining = traj->target_dist_mm - traj->position;

    if (remaining <= 2.0f)
    {
        traj->current_speed = 0.0f;
        traj->done          = true;
        return 0.0f;
    }

    float brake_dist = (traj->current_speed * traj->current_speed)
                       / (2.0f * traj->decel);

    if (remaining <= brake_dist)
    {
        float new_spd = traj->current_speed - (traj->decel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, SPD_CREEP, traj->current_speed);
    }
    else if (traj->current_speed < traj->peak_speed)
    {
        float new_spd = traj->current_speed + (traj->accel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, 0.0f, traj->peak_speed);
    }
    else
    {
        traj->current_speed = traj->peak_speed;
    }

    traj->position += traj->current_speed * CTRL_LOOP_DT;
    return traj->current_speed;
}

static float tick_scurve(Trajectory_t *traj)
{
    float remaining = traj->target_dist_mm - traj->position;

    if (remaining <= 2.0f)
    {
        traj->current_speed = 0.0f;
        traj->done          = true;
        return 0.0f;
    }

    float brake_dist = (traj->current_speed * traj->current_speed)
                       / (2.0f * traj->decel);

    if (remaining <= brake_dist)
    {
        float peak = (traj->peak_speed > 0.0f) ? traj->peak_speed : SPD_CREEP;
        float d_phase = (float)M_PI * traj->decel * CTRL_LOOP_DT / peak;
        traj->scurve_phase += d_phase;
        if (traj->scurve_phase > (float)M_PI)
        {
            traj->scurve_phase = (float)M_PI;
        }

        float inst_decel = traj->decel * sinf(traj->scurve_phase);
        float new_spd = traj->current_speed - (inst_decel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, SPD_CREEP, traj->current_speed);
    }
    else if (traj->current_speed < traj->peak_speed)
    {
        float peak = (traj->peak_speed > 0.0f) ? traj->peak_speed : SPD_CREEP;
        float d_phase = (float)M_PI * traj->accel * CTRL_LOOP_DT / peak;
        traj->scurve_phase += d_phase;
        if (traj->scurve_phase > (float)M_PI)
        {
            traj->scurve_phase = (float)M_PI;
        }

        float inst_accel = traj->accel * sinf(traj->scurve_phase);
        float new_spd = traj->current_speed + (inst_accel * CTRL_LOOP_DT);
        traj->current_speed = clamp_f(new_spd, 0.0f, traj->peak_speed);

        if (traj->current_speed >= traj->peak_speed)
        {
            traj->scurve_phase = 0.0f;
        }
    }
    else
    {
        traj->current_speed = traj->peak_speed;
        traj->scurve_phase  = 0.0f;
    }

    traj->position += traj->current_speed * CTRL_LOOP_DT;
    return traj->current_speed;
}

float trajectory_tick(Trajectory_t *traj)
{
    if (traj == NULL || traj->done) { return 0.0f; }

    if (traj->type == TRAJ_SCURVE)
    {
        return tick_scurve(traj);
    }
    return tick_trapezoid(traj);
}

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

void trajectory_search(Trajectory_t *traj, uint8_t cells)
{
    trajectory_init(traj,
                    (float)cells * CELL_WIDTH_MM,
                    SPD_SEARCH,
                    ACCEL_SEARCH,
                    DECEL_SEARCH);
}

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

void trajectory_return(Trajectory_t *traj, uint8_t cells)
{
    trajectory_init(traj,
                    (float)cells * CELL_WIDTH_MM,
                    SPD_RETURN,
                    ACCEL_SEARCH,
                    DECEL_SEARCH);
}

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

float trajectory_min_dist(float cruise_mmps, float accel, float decel)
{
    if (accel <= 0.0f || decel <= 0.0f) { return 0.0f; }
    float a_dist = (cruise_mmps * cruise_mmps) / (2.0f * accel);
    float d_dist = (cruise_mmps * cruise_mmps) / (2.0f * decel);
    return a_dist + d_dist;
}
