/**
 * @file    trajectory.h
 * @brief   Trapezoidal velocity profile generator — public API.
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

typedef enum
{
    TRAJ_TRAPEZOIDAL = 0U,
    TRAJ_SCURVE      = 1U,
} TrajType_t;

typedef struct
{
    float     target_dist_mm;
    float     cruise_speed;
    float     accel;
    float     decel;
    TrajType_t type;

    float     accel_dist;
    float     decel_dist;
    float     cruise_dist;
    float     peak_speed;

    float     current_speed;
    float     position;
    bool      done;

    float     scurve_phase;
    float     scurve_accel;
} Trajectory_t;

typedef struct
{
    float  current_speed;
    float  position_mm;
    float  remaining_mm;
    float  peak_speed;
    bool   done;
    uint8_t zone;
} TrajStatus_t;

void trajectory_init(Trajectory_t *traj,
                     float         dist_mm,
                     float         cruise_mmps,
                     float         accel,
                     float         decel);

void trajectory_init_scurve(Trajectory_t *traj,
                             float         dist_mm,
                             float         cruise_mmps,
                             float         accel,
                             float         decel);

float trajectory_tick(Trajectory_t *traj);

bool trajectory_is_done(const Trajectory_t *traj);

float trajectory_current_speed(const Trajectory_t *traj);

float trajectory_position(const Trajectory_t *traj);

float trajectory_remaining(const Trajectory_t *traj);

void trajectory_search(Trajectory_t *traj, uint8_t cells);

void trajectory_speedrun(Trajectory_t *traj, uint8_t cells, float speed_mmps);

void trajectory_return(Trajectory_t *traj, uint8_t cells);

void trajectory_get_status(const Trajectory_t *traj, TrajStatus_t *status);

float trajectory_min_dist(float cruise_mmps, float accel, float decel);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H */
