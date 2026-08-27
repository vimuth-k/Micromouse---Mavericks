/**
 * @file    pid.h
 * @brief   Generic discrete-time PID controller — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define PID_D_FILTER_MAX_TAPS   8U

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float   d_buf[PID_D_FILTER_MAX_TAPS];
    uint8_t d_buf_idx;
    uint8_t d_filter_n;
    float   d_buf_sum;

    float i_limit;
    float out_limit;

    float last_p;
    float last_i;
    float last_d;
    float last_output;
} PID_t;

typedef struct
{
    float p;
    float i;
    float d;
    float output;
    float error;
    float integral;
} PidTerms_t;

void pid_init(PID_t *pid,
              float  kp, float ki, float kd,
              float  i_limit, float out_limit);

void pid_init_ex(PID_t  *pid,
                 float   kp, float ki, float kd,
                 float   i_limit, float out_limit,
                 uint8_t d_filter_n);

void pid_reset(PID_t *pid);

float pid_update(PID_t *pid, float error);

void pid_set_gains(PID_t *pid, float kp, float ki, float kd);

void pid_set_output_limit(PID_t *pid, float out_limit);

void pid_get_terms(const PID_t *pid, PidTerms_t *terms);

bool pid_is_saturated(const PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
