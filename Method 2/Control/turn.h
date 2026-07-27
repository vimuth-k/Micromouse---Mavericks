/**
 * @file    turn.h
 * @brief   MicroMaze 3 · Heading tracker and path/move executor.
 * @details
 *   WHAT THIS MODULE IS — AND ISN'T
 *   ─────────────────────────────────────────────────────────────────────
 *   This is NOT the low-level pivot-turn PID controller — that already
 *   lives entirely inside motion.c (motion_turn_right/left/180(), the
 *   MOTION_TURNING state handler, and its own PID instance). And it is
 *   NOT the heading-diff-to-turn-type calculation — that already lives
 *   in path_optimizer.c (path_optimizer_turn_type()).
 *
 *   What's missing between those two pieces is the layer that actually
 *   *drives* a PathMove_t/OptPath_t sequence (path_optimizer.h's output
 *   format) by calling the right motion.c primitive for each entry,
 *   while tracking the robot's current absolute heading (DIR_N/E/S/W)
 *   as it goes — since move_forward() doesn't change heading but the
 *   three turn primitives do, and nothing else in the project tracks
 *   this running total. explorer.c (executing a path back to start
 *   after the goal) and speedrun.c (executing the optimised path at
 *   speed) are both expected consumers of this module.
 *
 *   ALIGN-BEFORE-TURN
 *   ─────────────────────────────────────────────────────────────────────
 *   motion_align_front()'s own doc comment recommends calling it before
 *   every turn to improve accuracy (a robot 3 mm off-centre accumulates
 *   ~0.95° of heading error per turn). turn_execute_move() does this
 *   automatically for every MOVE_TURN_* entry — callers don't need to
 *   remember it themselves.
 *
 *   SAFETY-TRIP AWARENESS
 *   ─────────────────────────────────────────────────────────────────────
 *   motion.c's blocking primitives now service the scheduler during
 *   their spin (see motion.c's BLOCKING + VOLATILE FLAG PROTOCOL note),
 *   so a safety_check() trip can call motion_stop() and cut a move
 *   short mid-flight. turn_execute_path() checks safety_is_tripped()
 *   after every move and stops walking the remaining path if it fires,
 *   rather than blindly continuing onto moves that assume the robot
 *   ended up somewhere it didn't.
 *
 *   DIP MODE 4 (TURN_TEST)
 *   ─────────────────────────────────────────────────────────────────────
 *   turn_run_test_sequence() executes a 90°, a 180°, and a 360°
 *   (composed as two 180s) turn in sequence for the operator to verify
 *   by eye. It does not attempt to report a numeric accuracy figure:
 *   motion.c resets the gyro yaw to 0° immediately after every turn
 *   completes (so the next move starts from a clean reference), which
 *   means the achieved angle isn't observable from outside motion.c
 *   once the blocking call returns — there is no data to report beyond
 *   "motion.c's own PID says it converged within TURN_TOLERANCE_DEG",
 *   which is trivially true of every completed turn and not a
 *   meaningful diagnostic.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef TURN_H
#define TURN_H

#include <stdint.h>
#include "error.h"
#include "path_optimizer.h"  /* PathMove_t, OptPath_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the heading tracker to DIR_N.
 * @details Matches the robot's assumed start pose (row=15, col=0,
 *          facing DIR_N — see maze.h). Call once during system
 *          bring-up, after motion_init().
 * @return MM_OK always.
 */
MmResult_t turn_init(void);

/**
 * @brief  Execute one move from an optimised path.
 *
 * @details Dispatches on move->type:
 *            MOVE_FORWARD     → move_forward(move->cells, move->speed)
 *            MOVE_TURN_RIGHT  → motion_align_front(); motion_turn_right()
 *            MOVE_TURN_LEFT   → motion_align_front(); motion_turn_left()
 *            MOVE_TURN_180    → motion_align_front(); motion_turn_180()
 *            MOVE_DONE        → no-op
 *          Updates the internal heading tracker for the three turn
 *          cases (MOVE_FORWARD doesn't change heading). Blocks for the
 *          duration of the underlying motion.c call.
 *
 * @warning Main-loop context only — never call from an ISR.
 *
 * @param  move  Move to execute. NULL is a no-op.
 */
void turn_execute_move(const PathMove_t *move);

/**
 * @brief  Execute every move in an optimised path, in order.
 *
 * @details Calls turn_execute_move() for each entry from index 0 up to
 *          (but not including) the first MOVE_DONE sentinel or
 *          path->count, whichever comes first. After each move, checks
 *          safety_is_tripped() — if a safety check fired and cut that
 *          move short, the remaining path is abandoned rather than
 *          executed against a robot that isn't where the plan assumes.
 *
 * @warning Main-loop context only. Blocks for the duration of the
 *          entire path — potentially many seconds for a full run.
 *
 * @param  path  Path to execute. NULL is a no-op.
 *
 * @return MM_OK           Every move executed to completion.
 * @return MM_ERR_PARAM    path was NULL.
 * @return MM_ERR_GENERAL  Execution stopped early due to a safety trip
 *                        (see safety_trip_reason() for why).
 */
MmResult_t turn_execute_path(const OptPath_t *path);

/**
 * @brief  Current tracked absolute heading.
 * @return DIR_N/E/S/W (see maze.h), as last set by turn_init(),
 *         turn_set_heading(), or updated by turn_execute_move().
 */
uint8_t turn_get_heading(void);

/**
 * @brief  Force the heading tracker to a specific value.
 * @details Call after maze_reset_position() (e.g. before a speedrun,
 *          when the robot is physically back at the start cell facing
 *          DIR_N) so turn.c's tracked heading matches reality again.
 * @param  heading  New heading (DIR_N/E/S/W).
 */
void turn_set_heading(uint8_t heading);

/**
 * @brief  Run the DIP Mode 4 diagnostic turn sequence: 90° right, 180°,
 *         then another 180° (360° total), for visual verification.
 * @details Shows progress on the OLED and beeps once per completed
 *          turn. See the file banner for why this doesn't report a
 *          numeric accuracy figure. Implemented as three calls to
 *          turn_execute_move(), so the heading tracker stays accurate
 *          afterward (the robot really has rotated 90° net from where
 *          it started — 90+180+180 = 450° = 90° mod 360°).
 * @warning Main-loop context only.
 * @return MM_OK always.
 */
MmResult_t turn_run_test_sequence(void);

#ifdef __cplusplus
}
#endif

#endif /* TURN_H */
