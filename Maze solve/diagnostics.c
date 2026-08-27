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
    buttons_update();

    while (!buttons_just_pressed())
    {
        IrSnapshot_t snap;
        ir_get_snapshot(&snap);

        oled_show_sensors(snap.diff, snap.wall_front, snap.wall_left, snap.wall_right,
                           snap.front_error, snap.side_error);

        scheduler_tick();
        buttons_update();
        HAL_Delay(DIAG_REFRESH_MS);
    }

    return MM_OK;
}

MmResult_t diagnostics_run_gyro_debug(void)
{
    buttons_update();

    while (!buttons_just_pressed())
    {
        float yaw    = imu_yaw_deg();
        float rate   = imu_gyro_dps();
        float offset = imu_get_gyro_offset();

        oled_show_gyro(yaw, rate, offset);

        scheduler_tick();
        buttons_update();
        HAL_Delay(DIAG_REFRESH_MS);
    }

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
    buttons_update();

    while (!buttons_just_pressed())
    {
        battery_update();
        float   volts   = battery_voltage();
        uint8_t percent = battery_percent();

        oled_show_battery(volts, percent);

        scheduler_tick();
        buttons_update();
        HAL_Delay(DIAG_REFRESH_MS);
    }

    return MM_OK;
}
