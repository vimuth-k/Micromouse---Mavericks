/**
 * @file    motion.h
 * @brief   Motion control — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

typedef enum
{
    MOTION_IDLE      = 0U,
    MOTION_FORWARD   = 1U,
    MOTION_TURNING   = 2U,
    MOTION_ALIGNING  = 3U,
    MOTION_BRAKING   = 4U,
} MotionState_t;

typedef struct
{
    MotionState_t state;
    float         target_speed;
    float         ramp_speed;
    float         meas_speed_l;
    float         meas_speed_r;
    float         position_mm;
    float         target_dist_mm;
    float         yaw_deg;
    float         turn_target_deg;
    float         tracking_err_mm;
    bool          is_done;
} MotionStatus_t;

MmResult_t motion_init(void);

void move_forward(uint8_t cells, float speed_mmps);

void motion_turn_right(void);

void motion_turn_left(void);

void motion_turn_180(void);

void motion_align_front(void);

void motion_stop(void);

void motion_set_speed(float speed_mmps);

void motion_1khz_tick(void);

MotionState_t motion_get_state(void);

bool motion_is_idle(void);

void motion_get_status(MotionStatus_t *status);

float motion_speed_left_mmps(void);

float motion_speed_right_mmps(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */
