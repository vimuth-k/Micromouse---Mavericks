/**
 * @file    wall_follow.h
 * @brief   MicroMaze 3 · Left-hand-rule fallback maze solver (DIP Mode 5).
 * @details
 *   WHAT THIS MODULE IS — AND ISN'T
 *   ─────────────────────────────────────────────────────────────────────
 *   This is NOT the wall-centering PID that keeps the robot centred in
 *   a corridor while driving straight — that's layer 4 of motion.c's
 *   1 kHz control loop and runs on every move regardless of which
 *   algorithm picked the move. This module is the maze-solving
 *   *algorithm* itself: at each cell, apply the classic left-hand rule
 *   (try left, then straight, then right, then U-turn) using the
 *   current IR wall readings, execute the resulting move, and repeat
 *   until the goal is reached.
 *
 *   It's a simpler, memoryless alternative to the flood-fill approach
 *   in floodfill.c/explorer.c (not yet built) — no distance map, no
 *   BFS, just a per-cell reactive decision — hence "fallback": useful
 *   as a demonstration mode, a sanity check independent of the
 *   flood-fill implementation, or a get-to-the-goal-somehow option if
 *   flood-fill exploration is unavailable. It is not guaranteed to find
 *   the goal on every possible maze topology (see the
 *   WALL_FOLLOW_MAX_CELLS note below), unlike a correct flood-fill
 *   search.
 *
 *   STATE OWNERSHIP
 *   ─────────────────────────────────────────────────────────────────────
 *   Uses maze.c's own position/heading tracker (the extern `maze`
 *   MazeState_t global — robot_row/robot_col/robot_heading) rather than
 *   turn.c's simpler heading-only tracker, since this algorithm needs
 *   full (row, col) position to check maze_is_goal() every cell. It
 *   also opportunistically records every wall it senses via
 *   maze_update_walls_from_sensors(), so a wall-follower run still
 *   leaves useful map data behind for a later flood-fill speedrun even
 *   though it didn't build the map on purpose.
 *
 *   turn.c is deliberately NOT used here — it drives PathMove_t
 *   sequences with no concept of (row, col) position, which is exactly
 *   what this cell-by-cell reactive algorithm needs. Physical motion
 *   goes straight through motion.c; maze.c's maze_advance()/
 *   maze_turn_*() functions keep the position/heading bookkeeping in
 *   sync alongside it.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the wall-follower module.
 * @details Currently stateless between runs (all state lives in maze.c's
 *          own tracker, reset separately by maze_init()/
 *          maze_set_position()) — kept as its own init call for
 *          consistency with every other module's lifecycle and as the
 *          natural place to add wall-follower-specific state later.
 * @return MM_OK always.
 */
MmResult_t wall_follow_init(void);

/**
 * @brief  Run the left-hand-rule solver from the robot's current
 *         tracked position until the goal is reached or
 *         WALL_FOLLOW_MAX_CELLS is exceeded.
 *
 * @details Each cell: record sensed walls into the maze map, check for
 *          goal, check for a safety trip, then apply the left-hand
 *          rule — turn left if clear, else go straight if clear, else
 *          turn right if clear, else U-turn — calling
 *          motion_align_front() before any turn and advancing both the
 *          physical robot (motion.c) and the tracked position (maze.c)
 *          together.
 *
 * @warning Main-loop context only. Blocks until the goal is reached,
 *          the cell limit is hit, or a safety trip aborts the run —
 *          potentially the whole 8-minute run-timeout in the worst case.
 *
 * @return MM_OK           Goal reached.
 * @return MM_ERR_GENERAL  Aborted — either WALL_FOLLOW_MAX_CELLS was
 *                        exceeded without finding the goal, or a
 *                        safety trip fired (see safety_trip_reason()).
 */
MmResult_t wall_follow_run(void);

#ifdef __cplusplus
}
#endif

#endif /* WALL_FOLLOW_H */
