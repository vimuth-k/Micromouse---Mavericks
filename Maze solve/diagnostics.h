/**
 * @file    diagnostics.h
 * @brief   MicroMaze 3 · Live diagnostic view modes (DIP Modes 0, 11, 12, 13).
 * @details
 *   WHAT THIS MODULE DOES — AND DOESN'T
 *   ─────────────────────────────────────────────────────────────────────
 *   Every display primitive this module needs already exists —
 *   oled_show_sensors(), oled_show_gyro(), oled_show_battery() were
 *   clearly built for exactly these four DIP modes, and maze_print()
 *   already handles the ASCII wall-map dump. diagnostics.c contributes
 *   no new display logic of its own; it's the live-loop orchestration
 *   that reads fresh sensor data, calls the right oled_show_*()
 *   function, once per
 *   DIAG_REFRESH_MS, until the operator presses the button to exit
 *   (except MODE_PRINT_MAZE, which is a one-shot dump — a wall map
 *   doesn't change from one moment to the next while the robot is
 *   sitting still on a bench).
 *
 *     MODE_MONITOR (0)       diagnostics_run_monitor()
 *     MODE_GYRO_DEBUG (11)   diagnostics_run_gyro_debug()
 *     MODE_PRINT_MAZE (12)   diagnostics_print_maze()       — one-shot
 *     MODE_BATTERY_CHECK (13) diagnostics_run_battery_check()
 *
 *   Called from modes_run_monitor() / modes_run_gyro_debug() /
 *   modes_run_print_maze() / modes_run_battery_check() respectively —
 *   same one-DIP-mode-per-modes.c-wrapper pattern as every other module
 *   in this project (calibration.c, wall_follow.c, ...).
 *
 *   Gyro data needs no manual driving here: motion_1khz_tick() (running
 *   continuously off the TIM5 ISR once motion_init() has started it)
 *   already calls imu_update() every tick regardless of motion state,
 *   so imu_yaw_deg()/imu_gyro_dps() are always fresh to read.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the diagnostics module.
 * @details Currently stateless. Kept as its own init call for
 *          lifecycle consistency with every other module.
 * @return MM_OK always.
 */
MmResult_t diagnostics_init(void);

/**
 * @brief  MODE_MONITOR — live IR sensor view on OLED.
 * @details Refreshes oled_show_sensors() every DIAG_REFRESH_MS until the
 *          button is pressed.
 * @warning Main-loop context only. Blocks until the button is pressed.
 * @return MM_OK always (exits cleanly on button press).
 */
MmResult_t diagnostics_run_monitor(void);

/**
 * @brief  MODE_GYRO_DEBUG — live yaw / angular rate / offset view.
 * @details Refreshes oled_show_gyro() every DIAG_REFRESH_MS until the
 *          button is pressed.
 * @warning Main-loop context only. Blocks until the button is pressed.
 * @return MM_OK always.
 */
MmResult_t diagnostics_run_gyro_debug(void);

/**
 * @brief  MODE_PRINT_MAZE — one-shot ASCII dump of the known wall map
 *         and flood-fill distances.
 * @details Runs floodfill_run(FLOOD_TO_GOAL) so the distances
 *          reflect the current map, then calls maze_print().
 * @return MM_OK always.
 */
MmResult_t diagnostics_print_maze(void);

/**
 * @brief  MODE_BATTERY_CHECK — live battery voltage / percentage view.
 * @details Refreshes oled_show_battery() every DIAG_REFRESH_MS until
 *          the button is pressed. Does not itself decide low-battery
 *          policy (motors disabling, warnings) — that's battery.c's
 *          and safety.c's job; this is read-only display.
 * @warning Main-loop context only. Blocks until the button is pressed.
 * @return MM_OK always.
 */
MmResult_t diagnostics_run_battery_check(void);

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSTICS_H */
