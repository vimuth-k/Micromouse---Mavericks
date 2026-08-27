/**
 * @file    explorer.h
 * @brief   MicroMaze 3 · Flood-fill maze search and return-to-start (DIP Mode 6).
 * @details
 *   WHAT THIS MODULE IS — AND ISN'T
 *   ─────────────────────────────────────────────────────────────────────
 *   This is the actual BFS-guided exploration algorithm — the correct,
 *   complete counterpart to wall_follow.c's simpler (and not always
 *   sufficient) left-hand-rule fallback. It contains no maze-solving
 *   math of its own: floodfill.c already provides the BFS distance map
 *   and floodfill_best_direction(), maze.c already provides wall
 *   recording and position/heading tracking, and path_optimizer.c
 *   already provides the heading-diff-to-turn-type calculation
 *   (path_optimizer_turn_type()) that this module reuses rather than
 *   re-deriving. explorer.c's job is purely the per-cell loop that
 *   drives those three modules together against the physical robot via
 *   motion.c — the same integration pattern wall_follow.c uses, with
 *   floodfill's BFS decision in place of the left-hand rule.
 *
 *   TWO PHASES, TWO FUNCTIONS
 *   ─────────────────────────────────────────────────────────────────────
 *   explorer_search_to_goal() and explorer_return_to_start() are kept
 *   separate rather than combined into one "run a full search" call,
 *   because state_machine.h's design has SEARCHING and RETURNING as
 *   distinct states — the caller (modes_run_search(), not yet written)
 *   is expected to call state_machine_transition() between them. That
 *   orchestration deliberately does not live here; explorer.c only
 *   knows how to drive the robot, not what state the run is officially
 *   in.
 *
 *   RE-FLOODING EVERY CELL
 *   ─────────────────────────────────────────────────────────────────────
 *   Both phases call floodfill_run() again after every single move,
 *   whether or not a new wall was actually discovered at that cell.
 *   floodfill.c's own doc says a full 16×16 BFS takes well under 0.1 ms
 *   at 100 MHz, so the cost of always re-flooding is negligible next to
 *   a several-hundred-millisecond cell traversal, and it avoids a
 *   subtler bug class: tracking "did this cell reveal a wall we didn't
 *   already know about" correctly for every one of the 4 sensor
 *   readings is more moving parts for the same result.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef EXPLORER_H
#define EXPLORER_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the explorer module.
 * @details Currently stateless between runs — all persistent state
 *          lives in maze.c's tracker and floodfill.c's distance map,
 *          both reset independently. Kept as its own init call for
 *          lifecycle consistency with every other module, and as the
 *          natural place to add explorer-specific state later.
 * @return MM_OK always.
 */
MmResult_t explorer_init(void);

/**
 * @brief  Flood-fill search from the robot's current tracked position
 *         until any goal cell is reached.
 *
 * @details Each cell: record sensed walls, re-run floodfill_run()
 *          (FLOOD_TO_GOAL), check for goal, check for a safety trip,
 *          then move toward floodfill_best_direction() — turning via
 *          path_optimizer_turn_type() + motion.c, and keeping maze.c's
 *          position/heading tracker in sync exactly as wall_follow.c
 *          does.
 *
 * @warning Main-loop context only. Blocks until the goal is reached,
 *          EXPLORER_MAX_CELLS is hit, or a safety trip aborts the run.
 *
 * @return MM_OK           Goal reached.
 * @return MM_ERR_GENERAL  Aborted — EXPLORER_MAX_CELLS exceeded,
 *                        floodfill_best_direction() returned DIR_NONE
 *                        (maze-state inconsistency), or a safety trip
 *                        fired (see safety_trip_reason()).
 */
MmResult_t explorer_search_to_goal(void);

/**
 * @brief  Flood-fill navigation from the robot's current tracked
 *         position back to the start cell.
 *
 * @details Same per-cell loop as explorer_search_to_goal(), but seeded
 *          with FLOOD_TO_START and checking (START_ROW, START_COL)
 *          instead of maze_is_goal(). Still records sensed walls and
 *          re-floods every cell — the return leg mostly retraces known
 *          territory, but treating it identically to the search phase
 *          costs nothing and stays correct if reality doesn't quite
 *          match what was recorded going the other way.
 *
 * @warning Main-loop context only. Same blocking/failure semantics as
 *          explorer_search_to_goal().
 *
 * @return MM_OK           Back at the start cell.
 * @return MM_ERR_GENERAL  Aborted — see explorer_search_to_goal().
 */
MmResult_t explorer_return_to_start(void);

#ifdef __cplusplus
}
#endif

#endif /* EXPLORER_H */
