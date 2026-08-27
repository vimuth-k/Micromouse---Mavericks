/**
 * @file    modes.c
 * @brief   MicroMaze 3 · DIP-mode dispatch and execution implementation.
 * @details Direct dispatcher for all operating modes. Subsystems are called
 *          directly or via static helpers, with states coordinated and a final
 *          idle loop entered upon completion.
 *
 * @author  VDawn
 * @date    2026
 */
#include "modes.h"
#include "config.h"
#include "error.h"
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
 * @brief  Show a final result on the OLED, then loop forever.
 *
 * @details Services the scheduler while idling so battery monitoring
 *          keeps running even after the mode's task is done. Only a
 *          board reset gets out of this loop.
 *
 * @param  line1  First OLED line (result headline).
 * @param  line2  Second OLED line (detail, e.g. a measurement or PASS/FAIL).
 */
static void modes_idle_forever(const char *line1, const char *line2)
{
    oled_show_message(line1, line2);

    while (1)
    {
        scheduler_tick();
        HAL_Delay(500U);
    }
}

/**
 * @brief  Spin one motor briefly at STARTUP_TEST_PWM. Shared by
 *         run_motor_test()'s left/right steps.
 */
static void test_one_motor(const char *label, int32_t pwm_left, int32_t pwm_right)
{
    (void)label;
    encoders_reset();
    motors_enable();
    motors_set(pwm_left, pwm_right);
    HAL_Delay(STARTUP_TEST_MOTOR_MS);
    motors_coast();
}

/**
 * @brief  DIP Mode 2: Spin each motor in turn and verify encoder counts.
 */
static void run_motor_test(void)
{
    oled_show_message("MOTOR TEST", "LEFT...");
    test_one_motor("LEFT", STARTUP_TEST_PWM, 0);
    HAL_Delay(300U);

    oled_show_message("MOTOR TEST", "RIGHT...");
    test_one_motor("RIGHT", 0, STARTUP_TEST_PWM);

    modes_idle_forever("MOTOR TEST", "DONE");
}

/**
 * @brief  DIP Mode 3: Drive STRAIGHT_TEST_CELLS cells and report yaw drift.
 */
static void run_straight_test(void)
{
    oled_show_message("STRAIGHT TEST", "DRIVING...");

    move_forward(STRAIGHT_TEST_CELLS, SPD_SEARCH);
    float drift_deg = imu_yaw_deg();

    char line2[24];
    (void)snprintf(line2, sizeof(line2), "DRIFT %.1f DEG", drift_deg);
    modes_idle_forever("STRAIGHT TEST", line2);
}

/**
 * @brief  DIP Mode 6: Flood-fill search to the goal, then return to start.
 */
static void run_search(void)
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

/**
 * @brief  DIP Modes 7/8/9: Execute the known optimised path at fixed speed.
 */
static void run_speed(float speed_mmps)
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

/**
 * @brief  DIP Mode 10: Full qualifier sequence: search, return, then 3 speed runs.
 */
static void run_auto_qualifier(void)
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

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t modes_init(void)
{
    return MM_OK;
}

void modes_run(uint8_t mode)
{
    switch (mode)
    {
        case MODE_MONITOR:
            (void)diagnostics_run_monitor();
            modes_idle_forever("MONITOR", "DONE");
            break;

        case MODE_IR_CALIBRATE:
        {
            MmResult_t result = calibration_run_ir();
            modes_idle_forever("IR CALIBRATE", (result == MM_OK) ? "SAVED OK" : "FAILED");
            break;
        }

        case MODE_MOTOR_TEST:
            run_motor_test();
            break;

        case MODE_STRAIGHT_TEST:
            run_straight_test();
            break;

        case MODE_TURN_TEST:
            (void)turn_run_test_sequence();
            modes_idle_forever("TURN TEST", "DONE - CHECK");
            break;

        case MODE_WALL_FOLLOWER:
        {
            MmResult_t result = wall_follow_run();
            modes_idle_forever("WALL FOLLOW", (result == MM_OK) ? "GOAL REACHED" : "FAILED");
            break;
        }

        case MODE_SEARCH_RUN:
            run_search();
            break;

        case MODE_SPEED_RUN_1:
            run_speed(SPD_RUN1);
            break;

        case MODE_SPEED_RUN_2:
            run_speed(SPD_RUN2);
            break;

        case MODE_SPEED_RUN_3:
            run_speed(SPD_RUN3);
            break;

        case MODE_AUTO_QUALIFIER:
            run_auto_qualifier();
            break;

        case MODE_GYRO_DEBUG:
            (void)diagnostics_run_gyro_debug();
            modes_idle_forever("GYRO DEBUG", "DONE");
            break;

        case MODE_PRINT_MAZE:
            (void)diagnostics_print_maze();
            modes_idle_forever("PRINT MAZE", "DONE");
            break;

        case MODE_BATTERY_CHECK:
            (void)diagnostics_run_battery_check();
            modes_idle_forever("BATTERY CHECK", "DONE");
            break;

        case MODE_STARTUP_TEST:
        {
            StartupTestResult_t result;
            (void)startup_test_run(&result);
            modes_idle_forever("STARTUP TEST",
                               startup_test_all_passed(&result) ? "ALL PASS" : "FAIL");
            break;
        }

        case MODE_RESERVED:
        default:
            (void)diagnostics_run_monitor();
            modes_idle_forever("MONITOR", "DONE");
            break;
    }

    /* Fallback safety if any mode function ever returns */
    motors_disable();

    while (1)
    {
        HAL_Delay(500U);
    }
}

void modes_dispatch(uint8_t mode)
{
    modes_run(mode);
}
