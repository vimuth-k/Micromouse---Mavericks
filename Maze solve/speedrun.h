/**
 * @file    speedrun.h
 * @brief   MicroMaze 3 · Execute the optimised path at speed (DIP Modes
 *          7/8/9, and the Run1/2/3 legs of Mode 10 AUTO_QUALIFIER).
 * @details
 *   WHAT THIS MODULE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   Unlike explorer.c, this is a pure open-loop path executor — no
 *   per-cell sensing or floodfill re-runs, because the maze is assumed
 *   already known (either from a search earlier this session, or
 *   loaded from Flash at boot — see main.c's maze_load_from_flash()
 *   call and explorer.c's maze_save_to_flash() calls after each
 *   successful phase). This is exactly turn.c's intended use case:
 *
 *     1. floodfill_extract_path() — trace the shortest known route
 *        from START to the goal as a raw DIR_N/E/S/W sequence.
 *     2. path_optimizer_run() — merge consecutive forward steps and
 *        classify turns, producing an OptPath_t.
 *     3. turn_execute_path() — drive the whole thing, blind, at the
 *        requested speed.
 *
 *   No wall sensing happens during the run itself — if the physical
 *   maze has changed since it was mapped, this module has no way to
 *   notice (that's explorer.c's job, not this one's).
 *
 *   WHY THIS IS SEPARATE FROM EXPLORER.C
 *   ─────────────────────────────────────────────────────────────────────
 *   modes.c calls modes_run_speed(SPD_RUN1/2/3) for DIP Modes 7/8/9 —
 *   one handler, three different target speeds — and the same
 *   speedrun_run() would also back the repeated Run1→Run2→Run3 legs of
 *   Mode 10 (AUTO_QUALIFIER), reusing whatever explorer.c mapped during
 *   that same session's search leg.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef SPEEDRUN_H
#define SPEEDRUN_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Compute the optimised path from the currently-known maze map
 *         and execute it at @p speed_mmps.
 *
 * @details Resets the robot's tracked position to (START_ROW,
 *          START_COL, DIR_N) in both maze.c and turn.c before running —
 *          the physical robot is assumed to actually be back at the
 *          start cell when this is called (true for a fresh DIP-mode
 *          selection, and for each Run1/2/3 leg of AUTO_QUALIFIER, which
 *          returns to start between runs the same way explorer.c does).
 *
 * @warning Main-loop context only. Blocks for the duration of the
 *          entire run.
 *
 * @param  speed_mmps  Target cruise speed for the run (SPD_RUN1/2/3
 *                      from config.h, or any other value the caller
 *                      chooses).
 *
 * @return MM_OK           Path computed and executed successfully.
 * @return MM_ERR_NOT_FOUND No path to the goal exists in the currently
 *                        known map — run a search first (Mode 6).
 * @return MM_ERR_OVERFLOW  Optimised path exceeded PATH_MAX_MOVES.
 * @return MM_ERR_GENERAL   Execution stopped early — a safety trip
 *                        fired mid-run (see safety_trip_reason()).
 */
MmResult_t speedrun_run(float speed_mmps);

#ifdef __cplusplus
}
#endif

#endif /* SPEEDRUN_H */
