/**
 * @file    diagnostics.c
 * @brief   MicroMaze 3 · Live diagnostic view modes — implementation.
 * @details See diagnostics.h for the full design rationale — every
 *          display primitive used here already exists; this file is
 *          purely the live-loop orchestration around them.
 *
 * @author  VDawn
 * @date    2026
 */
#include "diagnostics.h"
#include "config.h"
#include "error.h"
#include "logger.h"
#include "oled.h"
#include "ir.h"
#include "imu.h"
#include "battery.h"
#include "buttons.h"
#include "maze.h"
#include "floodfill.h"
#include "scheduler.h"
#include "main.h"     /* HAL_Delay */

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t diagnostics_init(void)
{
    return MM_OK;
}

MmResult_t diagnostics_run_monitor(void)
{
    LOG_INFO("Monitor mode: press button to exit");

    const IrCalData_t *cal = ir_cal_get();
    LOG_INFO("IR thresholds: %u %u %u %u %u %u",
             cal->threshold[0], cal->threshold[1], cal->threshold[2],
             cal->threshold[3], cal->threshold[4], cal->threshold[5]);

    buttons_update();

    while (!buttons_just_pressed())
    {
        IrSnapshot_t snap;
        ir_get_snapshot(&snap);

        oled_show_sensors(snap.diff, snap.wall_front, snap.wall_left, snap.wall_right,
                           snap.front_error, snap.side_error);

        LOG_RAW("IR %4u %4u %4u %4u %4u %4u  F:%u L:%u R:%u  FErr:%.1f SErr:%.1f\r\n",
                snap.diff[0], snap.diff[1], snap.diff[2],
                snap.diff[3], snap.diff[4], snap.diff[5],
                (unsigned)snap.wall_front, (unsigned)snap.wall_left, (unsigned)snap.wall_right,
                snap.front_error, snap.side_error);

        scheduler_tick();
        buttons_update();
        HAL_Delay(DIAG_REFRESH_MS);
    }

    LOG_INFO("Monitor mode: exiting");
    return MM_OK;
}

MmResult_t diagnostics_run_gyro_debug(void)
{
    LOG_INFO("Gyro debug: press button to exit");

    buttons_update();

    while (!buttons_just_pressed())
    {
        float yaw    = imu_yaw_deg();
        float rate   = imu_gyro_dps();
        float offset = imu_get_gyro_offset();

        oled_show_gyro(yaw, rate, offset);
        LOG_RAW("YAW %.2f RATE %.2f OFFSET %.4f\r\n", yaw, rate, offset);

        scheduler_tick();
        buttons_update();
        HAL_Delay(DIAG_REFRESH_MS);
    }

    LOG_INFO("Gyro debug: exiting");
    return MM_OK;
}

MmResult_t diagnostics_print_maze(void)
{
    floodfill_run(FLOOD_TO_GOAL);

    const uint8_t *flood_flat = floodfill_get_map();
    maze_print((const uint8_t (*)[MAZE_SIZE])flood_flat);

    return MM_OK;
}

MmResult_t diagnostics_run_battery_check(void)
{
    LOG_INFO("Battery check: press button to exit");

    buttons_update();

    while (!buttons_just_pressed())
    {
        battery_update();
        float   volts   = battery_voltage();
        uint8_t percent = battery_percent();

        oled_show_battery(volts, percent);
        LOG_RAW("BATT %.2fV %u%%\r\n", volts, percent);

        scheduler_tick();
        buttons_update();
        HAL_Delay(DIAG_REFRESH_MS);
    }

    LOG_INFO("Battery check: exiting");
    return MM_OK;
}
