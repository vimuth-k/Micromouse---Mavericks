/**
 * @file    motion.c
 * @brief   Motion control — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "motion.h"
#include "config.h"
#include "pins.h"
#include "pid.h"
#include "motors.h"
#include "encoders.h"
#include "ir.h"
#include "imu.h"
#include "utils.h"
#include "scheduler.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/* =========================================================================
 * PRIVATE — PID INSTANCES
 * ======================================================================= */

static PID_t s_pid_spd_l;
static PID_t s_pid_spd_r;
static PID_t s_pid_straight;
static PID_t s_pid_heading;
static PID_t s_pid_turn;

/* =========================================================================
 * PRIVATE — FSM STATE
 * ======================================================================= */

static volatile MotionState_t s_state = MOTION_IDLE;
static volatile bool s_motion_done = false;

static volatile float s_target_speed = 0.0f;
static volatile float s_ramp_speed = 0.0f;
static volatile float s_target_dist_mm = 0.0f;
static volatile float s_turn_target_deg = 0.0f;
static volatile float s_accel = ACCEL_NORMAL;
static volatile float s_decel = DECEL_NORMAL;

static volatile float s_meas_spd_l = 0.0f;
static volatile float s_meas_spd_r = 0.0f;

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

static void reset_all_controllers(void)
{
    pid_reset(&s_pid_spd_l);
    pid_reset(&s_pid_spd_r);
    pid_reset(&s_pid_straight);
    pid_reset(&s_pid_heading);
    pid_reset(&s_pid_turn);
}

static float clamp_f(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float compute_ramp_speed(float remaining_mm)
{
    float brake_dist = (s_ramp_speed * s_ramp_speed) / (2.0f * s_decel);

    if (remaining_mm <= 2.0f)
    {
        return 0.0f;
    }
    else if (remaining_mm <= brake_dist)
    {
        float new_spd = s_ramp_speed - (s_decel * CTRL_LOOP_DT);
        return clamp_f(new_spd, SPD_CREEP, s_ramp_speed);
    }
    else if (s_ramp_speed < s_target_speed)
    {
        float new_spd = s_ramp_speed + (s_accel * CTRL_LOOP_DT);
        return clamp_f(new_spd, 0.0f, s_target_speed);
    }
    else
    {
        return s_target_speed;
    }
}

/* =========================================================================
 * PRIVATE — ISR HANDLERS
 * ======================================================================= */

static void handle_forward(void)
{
    float pos_mm    = enc_avg_mm();
    float remaining = s_target_dist_mm - pos_mm;

    s_ramp_speed = compute_ramp_speed(remaining);

    if (s_ramp_speed <= 0.0f && remaining <= 2.0f)
    {
        s_state = MOTION_BRAKING;
        return;
    }

    float spd_l = enc_left_speed_mmps();
    float spd_r = enc_right_speed_mmps();
    s_meas_spd_l = spd_l;
    s_meas_spd_r = spd_r;

    float pwm_l = pid_update(&s_pid_spd_l, s_ramp_speed - spd_l);
    float pwm_r = pid_update(&s_pid_spd_r, s_ramp_speed - spd_r);

    float enc_diff  = (float)(enc_left_count() - enc_right_count());
    float straight  = pid_update(&s_pid_straight, enc_diff);
    float heading   = pid_update(&s_pid_heading, -imu_yaw_deg());

    float wall = 0.0f;
    if (ir_wall_left() && ir_wall_right())
    {
        wall = KP_WALL_CENTER * ir_side_error();
    }

    float final_l = pwm_l - straight - heading - wall;
    float final_r = pwm_r + straight + heading + wall;

    final_l = clamp_f(final_l, 0.0f, (float)PWM_MAX);
    final_r = clamp_f(final_r, 0.0f, (float)PWM_MAX);

    motors_set((int32_t)final_l, (int32_t)final_r);
}

static void handle_turning(void)
{
    float yaw   = imu_yaw_deg();
    float error = s_turn_target_deg - yaw;

    float abs_error = (error < 0.0f) ? -error : error;
    if (abs_error < TURN_TOLERANCE_DEG)
    {
        s_state = MOTION_BRAKING;
        return;
    }

    float pwm = pid_update(&s_pid_turn, abs_error);
    if (pwm < TURN_PWM_MIN) { pwm = TURN_PWM_MIN; }

    if (error > 0.0f)
    {
        motors_set( (int32_t)pwm, -(int32_t)pwm);
    }
    else
    {
        motors_set(-(int32_t)pwm,  (int32_t)pwm);
    }
}

static void handle_aligning(void)
{
    if (!ir_wall_front())
    {
        s_state = MOTION_BRAKING;
        return;
    }

    float ferr = ir_front_error();
    float corr = KP_ALIGN * ferr;
    int32_t base = 400;

    int32_t pwm_l = (int32_t)clamp_f((float)base - corr, 0.0f, 800.0f);
    int32_t pwm_r = (int32_t)clamp_f((float)base + corr, 0.0f, 800.0f);
    motors_set(pwm_l, pwm_r);

    float abs_ferr = (ferr < 0.0f) ? -ferr : ferr;
    if (ir_wall_front_close() && abs_ferr < 40.0f)
    {
        s_state = MOTION_BRAKING;
    }
}

static void handle_braking(void)
{
    motors_brake();
    s_state       = MOTION_IDLE;
    s_motion_done = true;
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

MmResult_t motion_init(void)
{
    pid_init_ex(&s_pid_spd_l,
                KP_SPEED, KI_SPEED, KD_SPEED,
                SPEED_INTEG_LIMIT, (float)PWM_MAX, 4U);

    pid_init_ex(&s_pid_spd_r,
                KP_SPEED, KI_SPEED, KD_SPEED,
                SPEED_INTEG_LIMIT, (float)PWM_MAX, 4U);

    pid_init(&s_pid_straight,
             KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT,
             STRAIGHT_INTEG_LIMIT, STRAIGHT_OUTPUT_LIMIT);

    pid_init(&s_pid_heading,
             KP_HEADING, KI_HEADING, KD_HEADING,
             HEADING_INTEG_LIMIT, HEADING_OUTPUT_LIMIT);

    pid_init(&s_pid_turn,
             KP_TURN, KI_TURN, KD_TURN,
             TURN_INTEG_LIMIT, TURN_PWM_MAX);

    s_state       = MOTION_IDLE;
    s_motion_done = false;
    s_ramp_speed  = 0.0f;

    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — BLOCKING MOVE PRIMITIVES
 * ======================================================================= */

void move_forward(uint8_t cells, float speed_mmps)
{
    encoders_reset();
    imu_reset_yaw();
    reset_all_controllers();
    motors_enable();

    if (speed_mmps <= SPD_SEARCH)
    {
        s_accel = ACCEL_SEARCH;
        s_decel = DECEL_SEARCH;
    }
    else
    {
        s_accel = ACCEL_NORMAL;
        s_decel = DECEL_NORMAL;
    }

    s_target_dist_mm = (float)cells * CELL_WIDTH_MM;
    s_target_speed   = clamp_f(speed_mmps, SPD_CREEP, SPD_ABSOLUTE_MAX);
    s_ramp_speed     = 0.0f;
    s_motion_done    = false;
    s_state          = MOTION_FORWARD;

    while (!s_motion_done) { scheduler_tick(); }

    HAL_Delay(TURN_SETTLE_MS);
    motors_coast();
}

static void do_turn(float angle_deg)
{
    imu_reset_yaw();
    reset_all_controllers();
    motors_enable();

    s_turn_target_deg = angle_deg;
    s_motion_done     = false;
    s_state           = MOTION_TURNING;

    while (!s_motion_done) { scheduler_tick(); }

    HAL_Delay(TURN_SETTLE_MS);
    motors_coast();

    imu_reset_yaw();
}

void motion_turn_right(void)
{
    do_turn(+90.0f);
}

void motion_turn_left(void)
{
    do_turn(-90.0f);
}

void motion_turn_180(void)
{
    do_turn(+180.0f);
}

void motion_align_front(void)
{
    if (!ir_wall_front())
    {
        return;
    }

    reset_all_controllers();
    motors_enable();

    s_motion_done = false;
    s_state       = MOTION_ALIGNING;

    uint32_t deadline = HAL_GetTick() + 1000U;
    while (!s_motion_done && (HAL_GetTick() < deadline)) { scheduler_tick(); }

    if (!s_motion_done)
    {
        s_state       = MOTION_IDLE;
        s_motion_done = true;
    }

    motors_coast();
}

void motion_stop(void)
{
    s_state       = MOTION_IDLE;
    s_motion_done = true;
    motors_brake();
    HAL_Delay(TURN_SETTLE_MS);
    motors_coast();
    motors_disable();
}

/* =========================================================================
 * PUBLIC API — RUNTIME CONTROL
 * ======================================================================= */

void motion_set_speed(float speed_mmps)
{
    s_target_speed = clamp_f(speed_mmps, SPD_CREEP, SPD_ABSOLUTE_MAX);
}

/* =========================================================================
 * PUBLIC API — 1 KHz ISR ENTRY POINT
 * ======================================================================= */

void motion_1khz_tick(void)
{
    ir_update();
    imu_update();

    (void)enc_left_delta();
    (void)enc_right_delta();

    switch (s_state)
    {
        case MOTION_FORWARD:
            handle_forward();
            break;

        case MOTION_TURNING:
            handle_turning();
            break;

        case MOTION_ALIGNING:
            handle_aligning();
            break;

        case MOTION_BRAKING:
            handle_braking();
            break;

        case MOTION_IDLE:
        default:
            motors_coast();
            break;
    }
}

/* =========================================================================
 * PUBLIC API — STATUS / DIAGNOSTICS
 * ======================================================================= */

MotionState_t motion_get_state(void)
{
    return s_state;
}

bool motion_is_idle(void)
{
    return s_state == MOTION_IDLE;
}

void motion_get_status(MotionStatus_t *status)
{
    if (status == NULL) { return; }

    status->state           = s_state;
    status->target_speed    = s_target_speed;
    status->ramp_speed      = s_ramp_speed;
    status->meas_speed_l    = s_meas_spd_l;
    status->meas_speed_r    = s_meas_spd_r;
    status->position_mm     = enc_avg_mm();
    status->target_dist_mm  = s_target_dist_mm;
    status->yaw_deg         = imu_yaw_deg();
    status->turn_target_deg = s_turn_target_deg;
    status->tracking_err_mm = enc_tracking_error_mm();
    status->is_done         = s_motion_done;
}

float motion_speed_left_mmps(void)
{
    return s_meas_spd_l;
}

float motion_speed_right_mmps(void)
{
    return s_meas_spd_r;
}
