/**
 * @file    pid.c
 * @brief   Generic discrete-time PID controller — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "pid.h"
#include <string.h>

static float clamp_sym(float x, float limit)
{
    if (x >  limit) { return  limit; }
    if (x < -limit) { return -limit; }
    return x;
}

static float d_filter_update(PID_t *pid, float d_raw)
{
    if (pid->d_filter_n <= 1U)
    {
        return d_raw;
    }

    pid->d_buf_sum -= pid->d_buf[pid->d_buf_idx];
    pid->d_buf[pid->d_buf_idx] = d_raw;
    pid->d_buf_sum += d_raw;

    pid->d_buf_idx = (uint8_t)((pid->d_buf_idx + 1U) % pid->d_filter_n);

    return pid->d_buf_sum / (float)pid->d_filter_n;
}

static void clear_state(PID_t *pid)
{
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->d_buf_idx   = 0U;
    pid->d_buf_sum   = 0.0f;
    pid->last_p      = 0.0f;
    pid->last_i      = 0.0f;
    pid->last_d      = 0.0f;
    pid->last_output = 0.0f;
    (void)memset(pid->d_buf, 0, sizeof(pid->d_buf));
}

void pid_init(PID_t *pid,
              float  kp, float ki, float kd,
              float  i_limit, float out_limit)
{
    pid_init_ex(pid, kp, ki, kd, i_limit, out_limit, 1U);
}

void pid_init_ex(PID_t  *pid,
                 float   kp, float ki, float kd,
                 float   i_limit, float out_limit,
                 uint8_t d_filter_n)
{
    if (pid == NULL) { return; }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->i_limit   = (i_limit   > 0.0f) ? i_limit   : 0.0f;
    pid->out_limit = (out_limit > 0.0f) ? out_limit : 0.0f;

    pid->d_filter_n = (d_filter_n < 1U) ? 1U :
                      (d_filter_n > PID_D_FILTER_MAX_TAPS)
                          ? (uint8_t)PID_D_FILTER_MAX_TAPS
                          : d_filter_n;

    clear_state(pid);
}

void pid_reset(PID_t *pid)
{
    if (pid == NULL) { return; }
    clear_state(pid);
}

float pid_update(PID_t *pid, float error)
{
    if (pid == NULL) { return 0.0f; }

    float p = pid->kp * error;

    pid->integral += error;
    pid->integral  = clamp_sym(pid->integral, pid->i_limit);
    float i        = pid->ki * pid->integral;

    float d_raw      = error - pid->prev_error;
    float d_filtered = d_filter_update(pid, d_raw);
    float d          = pid->kd * d_filtered;

    float raw_output = p + i + d;
    float output     = clamp_sym(raw_output, pid->out_limit);

    pid->prev_error = error;

    pid->last_p      = p;
    pid->last_i      = i;
    pid->last_d      = d;
    pid->last_output = raw_output;

    return output;
}

void pid_set_gains(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) { return; }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_output_limit(PID_t *pid, float out_limit)
{
    if (pid == NULL) { return; }
    pid->out_limit = (out_limit > 0.0f) ? out_limit : 0.0f;
}

void pid_get_terms(const PID_t *pid, PidTerms_t *terms)
{
    if (pid == NULL || terms == NULL) { return; }

    terms->p        = pid->last_p;
    terms->i        = pid->last_i;
    terms->d        = pid->last_d;
    terms->output   = pid->last_output;
    terms->error    = pid->prev_error;
    terms->integral = pid->integral;
}

bool pid_is_saturated(const PID_t *pid)
{
    if (pid == NULL)          { return false; }
    if (pid->i_limit <= 0.0f) { return false; }

    float abs_integral = (pid->integral < 0.0f)
                         ? -pid->integral
                         :  pid->integral;

    return abs_integral >= (pid->i_limit * 0.99f);
}
