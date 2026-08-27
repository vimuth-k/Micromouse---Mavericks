/**
 * @file    startup_test.c
 * @brief   MicroMaze 3 · Automated startup self-test — implementation.
 *
 * @author  VDawn
 * @date    2026
 */
#include "startup_test.h"
#include "battery.h"
#include "imu.h"
#include "ir.h"
#include "motors.h"
#include "encoders.h"
#include "oled.h"
#include "config.h"
#include "error.h"
#include "main.h"

static bool check_battery(void)
{
    float v = battery_get_voltage();
    return (v >= BATT_WARN_V);
}

static bool check_imu(void)
{
    return (imu_who_am_i() == MPU6500_WHO_AM_I_VAL);
}

static bool check_ir(void)
{
    ir_update();

    IrSnapshot_t snap;
    ir_get_snapshot(&snap);

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        if (snap.ambient[i] > 4000U)
        {
            return false;
        }
    }
    return true;
}

static bool check_motors_and_encoders(void)
{
    encoders_reset();
    motors_enable();

    motor_left_set((int32_t)PWM_DEADBAND + 200);
    motor_right_set((int32_t)PWM_DEADBAND + 200);
    HAL_Delay(30U);

    int32_t dl = enc_left_count();
    int32_t dr = enc_right_count();

    motor_left_set(0);
    motor_right_set(0);
    motors_brake();
    HAL_Delay(20U);
    motors_coast();
    motors_disable();

    if (dl < 0) { dl = -dl; }
    if (dr < 0) { dr = -dr; }

    return (dl > 0) && (dr > 0);
}

MmResult_t startup_test_run(void)
{
    bool batt_ok = check_battery();
    bool imu_ok  = check_imu();
    bool ir_ok   = check_ir();
    bool mot_ok  = check_motors_and_encoders();

    bool all_ok = batt_ok && imu_ok && ir_ok && mot_ok;

    if (all_ok)
    {
        oled_show_message("SELF TEST", "ALL SYSTEMS OK");
        HAL_Delay(500U);
        return MM_OK;
    }

    const char *line2 = !batt_ok ? "BATT LOW/FAIL" :
                        !imu_ok  ? "IMU NOT FOUND" :
                        !ir_ok   ? "IR SATURATED"  : "ENC/MOT STALL";

    oled_show_message("TEST FAILED", line2);
    HAL_Delay(1000U);

    return MM_ERR_GENERAL;
}
