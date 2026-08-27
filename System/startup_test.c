/**
 * @file    startup_test.c
 * @brief   MicroMaze 3 · Full hardware self-test sequence — implementation.
 * @details See startup_test.h for the full rationale behind each of the
 *          five checks and why the motor/encoder test is the only one
 *          that physically moves the robot.
 *
 * @author  VDawn
 * @date    2026
 */
#include "startup_test.h"
#include "config.h"
#include "error.h"
#include "oled.h"
#include "battery.h"
#include "imu.h"
#include "ir.h"
#include "motors.h"
#include "encoders.h"
#include "main.h"     /* HAL_Delay */

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers — one per subsystem
 * ═══════════════════════════════════════════════════════════════════════ */

static bool check_battery(StartupTestResult_t *result)
{
    battery_update();
    result->battery_voltage_v = battery_voltage();
    return (result->battery_voltage_v >= BATT_CRITICAL_V);
}

static bool check_imu(void)
{
    return (imu_who_am_i() == MPU6500_WHO_AM_I_VAL);
}

/**
 * @brief  A reading pinned at either ADC rail (0 or 4095) indicates a
 *         disconnected, shorted, or saturated channel rather than a
 *         real measurement.
 */
static bool reading_at_rail(uint16_t v)
{
    return (v == 0U) || (v == 4095U);
}

static bool check_ir(void)
{
    IrSnapshot_t snap;
    ir_get_snapshot(&snap);

    bool ok = true;
    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        if (reading_at_rail(snap.ambient[i]) || reading_at_rail(snap.lit[i]))
        {
            ok = false;
        }
    }

    return ok;
}

static bool check_calibration(void)
{
    /* Harmless, read-only re-check — ir_cal_load() just re-validates
     * the magic word and re-copies the same Flash-mapped struct that
     * ir_init() already loaded (or fell back from) at boot. */
    return (ir_cal_load() == MM_OK);
}

/**
 * @brief  Brief low-power spin of both wheels; PASS if each encoder
 *         registers at least STARTUP_TEST_MIN_ENC_COUNTS.
 */
static void check_motors_and_encoders(StartupTestResult_t *result)
{
    encoders_reset();
    motors_enable();
    motors_set(STARTUP_TEST_PWM, STARTUP_TEST_PWM);
    HAL_Delay(STARTUP_TEST_MOTOR_MS);
    motors_coast();

    result->enc_left_delta  = enc_left_count();
    result->enc_right_delta = enc_right_count();

    int32_t abs_left  = (result->enc_left_delta  < 0) ? -result->enc_left_delta  : result->enc_left_delta;
    int32_t abs_right = (result->enc_right_delta < 0) ? -result->enc_right_delta : result->enc_right_delta;

    result->motor_left_ok  = (abs_left  >= STARTUP_TEST_MIN_ENC_COUNTS);
    result->motor_right_ok = (abs_right >= STARTUP_TEST_MIN_ENC_COUNTS);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t startup_test_run(StartupTestResult_t *result)
{
    if (result == NULL)
    {
        return MM_ERR_PARAM;
    }

    oled_show_message("SELF-TEST", "BATTERY...");
    result->battery_ok = check_battery(result);

    oled_show_message("SELF-TEST", "IMU...");
    result->imu_ok = check_imu();

    oled_show_message("SELF-TEST", "IR SENSORS...");
    result->ir_ok = check_ir();

    oled_show_message("SELF-TEST", "IR CAL...");
    result->calibration_ok = check_calibration();

    oled_show_message("SELF-TEST", "MOTORS...");
    check_motors_and_encoders(result);

    bool all_ok = startup_test_all_passed(result);

    if (all_ok)
    {
        oled_show_message("SELF-TEST", "ALL PASS");
    }
    else
    {
        oled_show_error("SELF-TEST FAIL", (int32_t)MM_ERR_GENERAL);
    }

    return MM_OK;
}

bool startup_test_all_passed(const StartupTestResult_t *result)
{
    if (result == NULL)
    {
        return false;
    }

    return result->battery_ok
        && result->imu_ok
        && result->ir_ok
        && result->calibration_ok
        && result->motor_left_ok
        && result->motor_right_ok;
}
