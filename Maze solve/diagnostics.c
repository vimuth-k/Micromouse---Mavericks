/**
 * @file    diagnostics.c
 * @brief   MicroMaze 3 · Live diagnostic view modes — implementation.
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
/* ── Internal: shared poll loop ─────────────────────────────────────── */
typedef void (*DiagRefresh_t)(void);
static void diag_refresh_sensors(void)
{
    IrSnapshot_t snap;
    ir_get_snapshot(&snap);
    oled_show_sensors(snap.diff, snap.wall_front, snap.wall_left, snap.wall_right,
                      snap.front_error, snap.side_error);
}
static void diag_refresh_gyro(void)
{
    oled_show_gyro(imu_yaw_deg(), imu_gyro_dps(), imu_get_gyro_offset());
}
static void diag_refresh_battery(void)
{
    battery_update();
    oled_show_battery(battery_voltage(), battery_percent());
}
/** Poll @p refresh every DIAG_REFRESH_MS until the button is pressed. */
static void diag_live_loop(DiagRefresh_t refresh)
{
    buttons_update();
    while (!buttons_just_pressed())
    {
        refresh();
        scheduler_tick();
        buttons_update();
        HAL_Delay(DIAG_REFRESH_MS);
    }
}
/* ── Public API ──────────────────────────────────────────────────────── */
MmResult_t diagnostics_run(uint8_t mode)
