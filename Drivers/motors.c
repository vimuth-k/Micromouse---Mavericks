/**
 * @file    motors.c
 * @brief   TB6612FNG dual H-bridge motor driver — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "motors.h"
#include "pins.h"
#include "config.h"
#include "main.h"
#include "utils.h"

static volatile int32_t s_pwm_left  = 0;
static volatile int32_t s_pwm_right = 0;
static bool s_enabled = false;

static void apply_left(int32_t speed)
{
    if (speed > 0)
    {
        MOTOR_L_FORWARD();
        MOTOR_L_CCR = (uint32_t)speed;
    }
    else if (speed < 0)
    {
        MOTOR_L_REVERSE();
        MOTOR_L_CCR = (uint32_t)(-speed);
    }
    else
    {
        MOTOR_L_COAST();
        MOTOR_L_CCR = 0U;
    }
}

static void apply_right(int32_t speed)
{
    if (speed > 0)
    {
        MOTOR_R_FORWARD();
        MOTOR_R_CCR = (uint32_t)speed;
    }
    else if (speed < 0)
    {
        MOTOR_R_REVERSE();
        MOTOR_R_CCR = (uint32_t)(-speed);
    }
    else
    {
        MOTOR_R_COAST();
        MOTOR_R_CCR = 0U;
    }
}

MmResult_t motors_init(void)
{
    MOTOR_L_COAST();
    MOTOR_R_COAST();

    MOTOR_L_CCR = 0U;
    MOTOR_R_CCR = 0U;

    MOTOR_DISABLE();
    s_enabled    = false;
    s_pwm_left   = 0;
    s_pwm_right  = 0;

    return MM_OK;
}

void motors_enable(void)
{
    MOTOR_ENABLE();
    s_enabled = true;
}

void motors_disable(void)
{
    MOTOR_L_CCR = 0U;
    MOTOR_R_CCR = 0U;
    MOTOR_L_COAST();
    MOTOR_R_COAST();

    MOTOR_DISABLE();
    s_enabled   = false;
    s_pwm_left  = 0;
    s_pwm_right = 0;
}

bool motors_is_enabled(void)
{
    return s_enabled;
}

void motor_left_set(int32_t speed)
{
    int32_t corrected = speed * LEFT_MOTOR_POL;
    corrected = CLAMP(corrected, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

    if ((corrected > 0) && (corrected < (int32_t)PWM_DEADBAND))
    {
        corrected = (int32_t)PWM_DEADBAND;
    }
    else if ((corrected < 0) && (corrected > -(int32_t)PWM_DEADBAND))
    {
        corrected = -(int32_t)PWM_DEADBAND;
    }

    apply_left(corrected);
    s_pwm_left = corrected;
}

void motor_right_set(int32_t speed)
{
    int32_t corrected = speed * RIGHT_MOTOR_POL;
    corrected = CLAMP(corrected, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

    if ((corrected > 0) && (corrected < (int32_t)PWM_DEADBAND))
    {
        corrected = (int32_t)PWM_DEADBAND;
    }
    else if ((corrected < 0) && (corrected > -(int32_t)PWM_DEADBAND))
    {
        corrected = -(int32_t)PWM_DEADBAND;
    }

    apply_right(corrected);
    s_pwm_right = corrected;
}

void motors_set(int32_t left, int32_t right)
{
    int32_t cl = left  * LEFT_MOTOR_POL;
    int32_t cr = right * RIGHT_MOTOR_POL;

    cl = CLAMP(cl, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);
    cr = CLAMP(cr, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

    if ((cl > 0) && (cl < (int32_t)PWM_DEADBAND)) { cl =  (int32_t)PWM_DEADBAND; }
    if ((cl < 0) && (cl > -(int32_t)PWM_DEADBAND)){ cl = -(int32_t)PWM_DEADBAND; }
    if ((cr > 0) && (cr < (int32_t)PWM_DEADBAND)) { cr =  (int32_t)PWM_DEADBAND; }
    if ((cr < 0) && (cr > -(int32_t)PWM_DEADBAND)){ cr = -(int32_t)PWM_DEADBAND; }

    apply_left(cl);
    apply_right(cr);

    s_pwm_left  = cl;
    s_pwm_right = cr;
}

void motors_coast(void)
{
    MOTOR_L_COAST();
    MOTOR_R_COAST();
    MOTOR_L_CCR = 0U;
    MOTOR_R_CCR = 0U;
    s_pwm_left  = 0;
    s_pwm_right = 0;
}

void motors_brake(void)
{
    MOTOR_L_BRAKE();
    MOTOR_R_BRAKE();
    MOTOR_L_CCR = (uint32_t)PWM_MAX;
    MOTOR_R_CCR = (uint32_t)PWM_MAX;
    s_pwm_left  = 0;
    s_pwm_right = 0;
}

void motors_get_state(MotorState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->pwm_left  = s_pwm_left;
    state->pwm_right = s_pwm_right;
    state->enabled   = s_enabled;
}
