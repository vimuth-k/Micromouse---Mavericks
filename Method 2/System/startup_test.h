/**
 * @file    startup_test.h
 * @brief   MicroMaze 3 · Full hardware self-test sequence (DIP Mode 14).
 * @details
 *   WHAT THIS MODULE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   An operator-triggered, comprehensive per-subsystem health check —
 *   distinct from the automatic checks main.c already runs at every
 *   boot (IMU WHO_AM_I, Flash calibration load). Selecting DIP Mode 14
 *   calls modes_run_startup_test(), which calls startup_test_run() here
 *   to exercise every subsystem in turn and report PASS/FAIL for each:
 *
 *     1. Battery    — battery_voltage() above BATT_CRITICAL_V.
 *     2. IMU         — imu_who_am_i() matches MPU6500_WHO_AM_I_VAL.
 *     3. IR sensors  — none of the 6 pairs stuck at the ADC rail (0 or
 *                      4095) on either the ambient or lit reading.
 *     4. Calibration — a valid IR calibration is loaded (ir_cal_get()
 *                      magic check via ir_cal_load()'s result at boot;
 *                      read-only, does not touch Flash again here).
 *     5. Motors +    — brief (STARTUP_TEST_MOTOR_MS) low-power
 *        encoders      (STARTUP_TEST_PWM) spin of both wheels; PASS if
 *                      both encoders register at least
 *                      STARTUP_TEST_MIN_ENC_COUNTS. The only test that
 *                      physically moves the robot — see the warning
 *                      below.
 *
 *   Buzzer and OLED are not independently tested (there is no read-back
 *   path for either), but the test sequence itself uses the OLED for
 *   step-by-step progress and ends with a buzzer pattern (success vs
 *   failure), so a completely dead OLED or buzzer is obvious to the
 *   operator without a formal PASS/FAIL line for it.
 *
 * @warning Test 5 drives both motors briefly. Place the robot on a
 *          stand or clear bench space before running Mode 14 — this is
 *          the same caution as Mode 2 (MOTOR_TEST).
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef STARTUP_TEST_H
#define STARTUP_TEST_H

#include <stdbool.h>
#include <stdint.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Per-subsystem pass/fail results from startup_test_run().
 */
typedef struct
{
    bool  battery_ok;        /**< Voltage above BATT_CRITICAL_V.          */
    bool  imu_ok;             /**< WHO_AM_I matched.                       */
    bool  ir_ok;               /**< No sensor pair stuck at the ADC rail.   */
    bool  calibration_ok;      /**< Valid IR calibration present in Flash.  */
    bool  motor_left_ok;       /**< Left encoder registered motion.         */
    bool  motor_right_ok;      /**< Right encoder registered motion.        */
    float battery_voltage_v;   /**< Measured value, for the OLED/log line.  */
    int32_t enc_left_delta;    /**< Encoder counts seen during the spin.    */
    int32_t enc_right_delta;   /**< Encoder counts seen during the spin.    */
} StartupTestResult_t;

/**
 * @brief  Run the full self-test sequence.
 *
 * @details Always completes and returns MM_OK — individual failures are
 *          reported in @p result rather than as a function-level error,
 *          matching the pattern established by gyro_cal_run(): a self-
 *          test's job is to report status, not to fail outright when a
 *          checked subsystem is unhealthy. Shows step-by-step progress
 *          on the OLED, logs each result via LOG_INFO/LOG_ERROR as it
 *          runs, and ends with an audible buzzer pattern — a distinct
 *          pattern for all-pass vs. any-fail.
 *
 * @warning Briefly drives both motors (see the file banner). Main-loop
 *          context only.
 *
 * @param  result  Populated with per-subsystem outcomes. Must not be
 *                  NULL.
 *
 * @return MM_OK        Sequence completed (regardless of individual
 *                      subsystem pass/fail — check @p result for that).
 * @return MM_ERR_PARAM  @p result was NULL; nothing was tested.
 */
MmResult_t startup_test_run(StartupTestResult_t *result);

/**
 * @brief  true if every field in @p result passed.
 * @param  result  Result populated by a prior startup_test_run() call.
 * @return true only if battery_ok, imu_ok, ir_ok, calibration_ok,
 *         motor_left_ok, and motor_right_ok are all true.
 */
bool startup_test_all_passed(const StartupTestResult_t *result);

#ifdef __cplusplus
}
#endif

#endif /* STARTUP_TEST_H */
