/**
 * @file    kinematics.c
 * @brief   Differential drive kinematics — implementation.
 *
 * All conversions are pure closed-form mathematical functions with no state.
 *
 * @author  VDawn
 * @date    2026
 */

#include "kinematics.h"
#include "config.h"
#include <math.h>

#ifndef KIN_PI
#define KIN_PI                  3.14159265f
#endif

static const float FF_PWM_PER_MMPS = (float)PWM_MAX / SPD_ABSOLUTE_MAX;
static const float MMPS_PER_COUNT_PER_TICK = MM_PER_COUNT * (float)CTRL_LOOP_HZ;
static const float HALF_TRACK = WHEEL_SPACING_MM * 0.5f;

/* -------------------------------------------------------------------------
 * ENCODER <-> DISTANCE / SPEED
 * ----------------------------------------------------------------------- */

float kin_counts_to_mm(int32_t counts)
{
    return (float)counts * MM_PER_COUNT;
}

int32_t kin_mm_to_counts(float dist_mm)
{
    float raw = dist_mm / MM_PER_COUNT;
    return (raw >= 0.0f) ? (int32_t)(raw + 0.5f) : (int32_t)(raw - 0.5f);
}

float kin_delta_to_mmps(int32_t delta_counts)
{
    return (float)delta_counts * MMPS_PER_COUNT_PER_TICK;
}

float kin_mmps_to_delta(float speed_mmps)
{
    if (MMPS_PER_COUNT_PER_TICK <= 0.0f) { return 0.0f; }
    return speed_mmps / MMPS_PER_COUNT_PER_TICK;
}

float kin_cells_to_mm(uint8_t cells)
{
    return (float)cells * CELL_WIDTH_MM;
}

uint8_t kin_mm_to_cells(float dist_mm)
{
    if (dist_mm <= 0.0f) { return 0U; }
    uint32_t c = (uint32_t)((dist_mm / CELL_WIDTH_MM) + 0.5f);
    return (c > 255U) ? 255U : (uint8_t)c;
}

/* -------------------------------------------------------------------------
 * WHEEL SPACE <-> ROBOT SPACE
 * ----------------------------------------------------------------------- */

void kin_wheels_to_robot(const WheelVelocity_t *wheels, RobotVelocity_t *robot)
{
    if (wheels == NULL || robot == NULL) { return; }
    robot->v_mmps    = (wheels->vl_mmps + wheels->vr_mmps) * 0.5f;
    robot->omega_rps = (wheels->vr_mmps - wheels->vl_mmps) / WHEEL_SPACING_MM;
    robot->omega_dps = robot->omega_rps * (180.0f / KIN_PI);
}

void kin_robot_to_wheels(const RobotVelocity_t *robot, WheelVelocity_t *wheels)
{
    if (robot == NULL || wheels == NULL) { return; }
    float half_omega_b = robot->omega_rps * HALF_TRACK;
    wheels->vl_mmps = robot->v_mmps - half_omega_b;
    wheels->vr_mmps = robot->v_mmps + half_omega_b;
}

/* -------------------------------------------------------------------------
 * SPEED <-> PWM (FEED-FORWARD)
 * ----------------------------------------------------------------------- */

int32_t kin_mmps_to_pwm(float speed_mmps)
{
    float pwm = speed_mmps * FF_PWM_PER_MMPS;
    if (pwm > (float)PWM_MAX)  { return (int32_t)PWM_MAX; }
    if (pwm < -(float)PWM_MAX) { return -(int32_t)PWM_MAX; }
    return (int32_t)pwm;
}

float kin_pwm_to_mmps(int32_t pwm)
{
    if (FF_PWM_PER_MMPS <= 0.0f) { return 0.0f; }
    return (float)pwm / FF_PWM_PER_MMPS;
}

/* -------------------------------------------------------------------------
 * TURN GEOMETRY
 * ----------------------------------------------------------------------- */

void kin_pivot_wheel_speeds(float omega_dps, WheelVelocity_t *wheels)
{
    if (wheels == NULL) { return; }
    float omega_rps = omega_dps * (KIN_PI / 180.0f);
    float v_wheel   = omega_rps * HALF_TRACK;
    wheels->vl_mmps = -v_wheel;
    wheels->vr_mmps =  v_wheel;
}

void kin_arc_wheel_speeds(float v_mmps, float omega_dps, WheelVelocity_t *wheels)
{
    if (wheels == NULL) { return; }
    float omega_rps    = omega_dps * (KIN_PI / 180.0f);
    float half_omega_b = omega_rps * HALF_TRACK;
    wheels->vl_mmps    = v_mmps - half_omega_b;
    wheels->vr_mmps    = v_mmps + half_omega_b;
}

float kin_turn_radius(float v_mmps, float omega_dps)
{
    if (omega_dps == 0.0f) { return 1e6f; }
    float omega_rps = omega_dps * (KIN_PI / 180.0f);
    return v_mmps / omega_rps;
}

float kin_turn_rate_for_time(float time_s)
{
    if (time_s <= 0.0f) { return 0.0f; }
    return 90.0f / time_s;
}

/* -------------------------------------------------------------------------
 * ODOMETRY & ANGLES
 * ----------------------------------------------------------------------- */

float kin_heading_change_deg(float dist_l_mm, float dist_r_mm)
{
    float delta_rad = (dist_r_mm - dist_l_mm) / WHEEL_SPACING_MM;
    return delta_rad * (180.0f / KIN_PI);
}

float kin_pivot_arc_mm(float angle_deg)
{
    float angle_rad = fabsf(angle_deg) * (KIN_PI / 180.0f);
    return angle_rad * HALF_TRACK;
}

float kin_deg_to_rad(float deg)
{
    return deg * (KIN_PI / 180.0f);
}

float kin_rad_to_deg(float rad)
{
    return rad * (180.0f / KIN_PI);
}

float kin_normalise_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f)  { angle_deg -= 360.0f; }
    while (angle_deg <= -180.0f) { angle_deg += 360.0f; }
    return angle_deg;
}
