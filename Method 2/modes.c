/**
 * @file    modes.c
 * @brief   MicroMaze 3 · DIP-mode dispatch layer — implementation.
 * @details See modes.h for the full design rationale, including the
 *          "never return" contract every function here follows via
 *          modes_idle_forever().
 *
 * @author  VDawn
 * @date    2026
 */
#include "modes.h"
#include "config.h"
#include "error.h"
#include "logger.h"
#include "oled.h"
#include "motors.h"
#include "encoders.h"
#include "motion.h"
#include "imu.h"
#include "turn.h"
#include "explorer.h"
#include "speedrun.h"
#include "wall_follow.h"
#include "calibration.h"
#include "diagnostics.h"
#include "startup_test.h"
#include "state_machine.h"
#include "scheduler.h"
#include "main.h"     /* HAL_Delay */
#include "pins.h"
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Show a final result on the OLED, log it, then loop forever.
 *
 * @details Every modes_run_*() function ends here instead of returning
 *          — see the "never return" contract in modes.h. Services the
 *          scheduler while idling so battery monitoring and log flushing
 *          keep running even after the mode's task is done. Only a
 *          board reset gets out of this loop.
 *
 * @param  line1  First OLED line (result headline).
 * @param  line2  Second OLED line (detail, e.g. a measurement or PASS/FAIL).
 */
static void modes_idle_forever(const char *line1, const char *line2)
{
    oled_show_message(line1, line2);
    LOG_INFO("Mode complete: %s / %s — reset the board to select a different mode",
             line1, line2);

    while (1)
    {
        scheduler_tick();
        HAL_Delay(500U);
    }
}

/**
 * @brief  Spin one motor briefly at STARTUP_TEST_PWM and report its
 *         encoder delta. Shared by modes_run_motor_test()'s left/right
 *         steps.
 */
static void test_one_motor(const char *label, int32_t pwm_left, int32_t pwm_right)
{
    encoders_reset();
    motors_enable();
    motors_set(pwm_left, pwm_right);
    HAL_Delay(STARTUP_TEST_MOTOR_MS);
    motors_coast();

    int32_t left  = enc_left_count();
    int32_t right = enc_right_count();

    LOG_INFO("Motor test (%s): left=%ld counts, right=%ld counts",
             label, (long)left, (long)right);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t modes_init(void)
{
    return MM_OK;
}

/**
 * @brief  Run one subsystem function and report success/failure on
 *         the OLED via modes_idle_forever().
 *
 * @details Shared by every DIP mode whose entire body is "call one
 *          subsystem function, then show the result" — collapses
 *          what used to be 7 near-identical (call, discard result,
 *          idle_forever) bodies into one real implementation plus a
 *          one-line caller per mode. Since the result was already
 *          being computed and just thrown away before, these modes
 *          now genuinely report failure instead of always claiming
 *          success regardless of what happened.
 *
 * @param  fn       Subsystem function to run.
 * @param  title    OLED line 1 (mode name).
 * @param  ok_msg   OLED line 2 on MM_OK.
 * @param  fail_msg OLED line 2 on any other result.
 */
static void run_simple_mode(MmResult_t (*fn)(void), const char *title,
                             const char *ok_msg, const char *fail_msg)
{
    modes_idle_forever(title, (fn() == MM_OK) ? ok_msg : fail_msg);
}

void modes_run_monitor(void)
{
    run_simple_mode(diagnostics_run_monitor, "MONITOR", "DONE", "FAILED");
}

void modes_run_ir_calibrate(void)
{
    run_simple_mode(calibration_run_ir, "IR CALIBRATE", "SAVED OK", "FAILED");
}

void modes_run_motor_test(void)
{
    LOG_INFO("Motor test: spinning left motor");
    oled_show_message("MOTOR TEST", "LEFT...");
    test_one_motor("LEFT", STARTUP_TEST_PWM, 0);
    HAL_Delay(300U);

    LOG_INFO("Motor test: spinning right motor");
    oled_show_message("MOTOR TEST", "RIGHT...");
    test_one_motor("RIGHT", 0, STARTUP_TEST_PWM);

    modes_idle_forever("MOTOR TEST", "DONE - SEE LOG");
}

void modes_run_straight_test(void)
{
    LOG_INFO("Straight test: driving %u cells at %.0f mm/s",
             (unsigned)STRAIGHT_TEST_CELLS, SPD_SEARCH);
    oled_show_message("STRAIGHT TEST", "DRIVING...");

    move_forward(STRAIGHT_TEST_CELLS, SPD_SEARCH);
    float drift_deg = imu_yaw_deg();

    LOG_INFO("Straight test: complete, yaw drift = %.2f deg", drift_deg);

    char line2[24];
    (void)snprintf(line2, sizeof(line2), "DRIFT %.1f DEG", drift_deg);
    modes_idle_forever("STRAIGHT TEST", line2);
}

void modes_run_turn_test(void)
{
    run_simple_mode(turn_run_test_sequence, "TURN TEST", "DONE - CHECK", "ABORTED");
}

void modes_run_wall_follower(void)
{
    run_simple_mode(wall_follow_run, "WALL FOLLOW", "GOAL REACHED", "FAILED");
}

void modes_run_search(void)
{
    (void)state_machine_transition(ROBOT_STATE_SEARCHING);
    MmResult_t result = explorer_search_to_goal();

    if (result == MM_OK)
    {
        (void)state_machine_transition(ROBOT_STATE_RETURNING);
        result = explorer_return_to_start();
    }

    if (result == MM_OK)
    {
        (void)state_machine_transition(ROBOT_STATE_IDLE);
        modes_idle_forever("SEARCH", "COMPLETE");
    }
    else
    {
        (void)state_machine_transition(ROBOT_STATE_ERROR);
        modes_idle_forever("SEARCH", "FAILED");
    }
}

void modes_run_speed(float speed_mmps)
{
    (void)state_machine_transition(ROBOT_STATE_SPEEDRUNNING);
    MmResult_t result = speedrun_run(speed_mmps);

    if (result == MM_OK)
    {
        (void)state_machine_transition(ROBOT_STATE_IDLE);
        modes_idle_forever("SPEEDRUN", "COMPLETE");
    }
    else
    {
        (void)state_machine_transition(ROBOT_STATE_ERROR);
        modes_idle_forever("SPEEDRUN", "FAILED");
    }
}

void modes_run_auto_qualifier(void)
{
    (void)state_machine_transition(ROBOT_STATE_SEARCHING);
    MmResult_t result = explorer_search_to_goal();

    if (result == MM_OK)
    {
        (void)state_machine_transition(ROBOT_STATE_RETURNING);
        result = explorer_return_to_start();
    }

    if (result != MM_OK)
    {
        (void)state_machine_transition(ROBOT_STATE_ERROR);
        modes_idle_forever("AUTO QUALIFIER", "SEARCH FAILED");
        return;
    }

    (void)state_machine_transition(ROBOT_STATE_IDLE);

    const float speeds[3] = { SPD_RUN1, SPD_RUN2, SPD_RUN3 };

    for (uint8_t i = 0U; i < 3U; i++)
    {
        (void)state_machine_transition(ROBOT_STATE_SPEEDRUNNING);
        result = speedrun_run(speeds[i]);

        if (result != MM_OK)
        {
            (void)state_machine_transition(ROBOT_STATE_ERROR);

            char line2[24];
            (void)snprintf(line2, sizeof(line2), "RUN %u FAILED", (unsigned)(i + 1U));
            modes_idle_forever("AUTO QUALIFIER", line2);
            return;
        }

        bool is_last = (i == 2U);
        (void)state_machine_transition(is_last ? ROBOT_STATE_FINISHED : ROBOT_STATE_IDLE);
    }

    modes_idle_forever("AUTO QUALIFIER", "ALL RUNS DONE");
}

void modes_run_gyro_debug(void)
{
    run_simple_mode(diagnostics_run_gyro_debug, "GYRO DEBUG", "DONE", "FAILED");
}

void modes_run_print_maze(void)
{
    run_simple_mode(diagnostics_print_maze, "PRINT MAZE", "SEE UART LOG", "FAILED");
}

void modes_run_battery_check(void)
{
    run_simple_mode(diagnostics_run_battery_check, "BATTERY CHECK", "DONE", "FAILED");
}

void modes_run_startup_test(void)
{
    StartupTestResult_t result;
    (void)startup_test_run(&result);
    modes_idle_forever("STARTUP TEST",
                        startup_test_all_passed(&result) ? "ALL PASS" : "SEE UART LOG");
}
