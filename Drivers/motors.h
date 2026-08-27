/**
 * @file    motors.h
 * @brief   TB6612FNG dual H-bridge motor driver — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef MOTORS_H
#define MOTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

typedef enum
{
    MOTOR_LEFT  = 0U,
    MOTOR_RIGHT = 1U,
} Motor_t;

typedef struct
{
    int32_t  pwm_left;
    int32_t  pwm_right;
    bool     enabled;
} MotorState_t;

MmResult_t motors_init(void);
void       motors_enable(void);
void       motors_disable(void);
bool       motors_is_enabled(void);

void       motor_left_set(int32_t speed);
void       motor_right_set(int32_t speed);
void       motors_set(int32_t left, int32_t right);

void       motors_coast(void);
void       motors_brake(void);

void       motors_get_state(MotorState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
