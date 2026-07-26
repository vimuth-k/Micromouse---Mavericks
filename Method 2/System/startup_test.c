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
#include "logger.h"
#include "oled.h"
#include "buzzer.h"
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
    bool ok = (result->battery_voltage_v >= BATT_CRITICAL_V);

    LOG_INFO("Self-test: battery %.2f V — %s",
             result->battery_voltage_v, ok ? "PASS" : "FAIL");
    return ok;
}

static bool check_imu(void)
{
    bool ok = (imu_who_am_i() == MPU6500_WHO_AM_I_VAL);

    LOG_INFO("Self-test: IMU WHO_AM_I 0x%02X (expected 0x%02X) — %s",
             imu_who_am_i(), MPU6500_WHO_AM_I_VAL, ok ? "PASS" : "FAIL");
    return ok;
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
            LOG_ERROR("Self-test: IR pair %u stuck at ADC rail "
                       "(ambient=%u lit=%u)", i, snap.ambient[i], snap.lit[i]);
            ok = false;
        }
    }

    LOG_INFO("Self-test: IR sensors — %s", ok ? "PASS" : "FAIL");
    return ok;
}

static bool check_calibration(void)
{
    /* Harmless, read-only re-check — ir_cal_load() just re-validates
     * the magic word and re-copies the same Flash-mapped struct that
     * ir_init() already loaded (or fell back from) at boot. */
    bool ok = (ir_cal_load() == MM_OK);

    LOG_INFO("Self-test: IR calibration present — %s", ok ? "PASS" : "FAIL");
    return ok;
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

    LOG_INFO("Self-test: left encoder %ld counts — %s",
             (long)result->enc_left_delta, result->motor_left_ok ? "PASS" : "FAIL");
    LOG_INFO("Self-test: right encoder %ld counts — %s",
             (long)result->enc_right_delta, result->motor_right_ok ? "PASS" : "FAIL");
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

    LOG_INFO("===== Startup self-test beginning =====");
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
    LOG_INFO("===== Startup self-test %s =====", all_ok ? "PASSED" : "FAILED");

    if (all_ok)
    {
        oled_show_message("SELF-TEST", "ALL PASS");
        buzzer_calibrated();
    }
    else
    {
        oled_show_error("SELF-TEST FAIL", (int32_t)MM_ERR_GENERAL);
        buzzer_error();
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
