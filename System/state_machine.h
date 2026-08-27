/**
 * @file    state_machine.h
 * @brief   MicroMaze 3 · Robot-level run state machine (idle / search /
 *          return / speedrun / finished / error).
 * @details
 *   WHAT THIS MODULE IS — AND ISN'T
 *   ─────────────────────────────────────────────────────────────────────
 *   This is NOT motion.c's MotionState_t (IDLE/FORWARD/TURNING/ALIGNING/
 *   BRAKING — the low-level motor FSM for a single move) and it is NOT
 *   the DIP-switch mode dispatcher in main.c (MODE_SEARCH_RUN etc. —
 *   which top-level function to call). It sits one layer above both:
 *   a single run through the maze, from the operator's point of view,
 *   moves through a small number of coarse-grained states.
 *
 *   This mirrors the MODE_AUTO_QUALIFIER sequence from config.h Section
 *   14 (DIP 10: "Search → Run1 → Run2 → Run3"), and Calibration/modes.c
 *   (not yet written) is the expected caller — modes_run_search() and
 *   modes_run_auto_qualifier() drive this FSM as they orchestrate a run,
 *   while modes_run_motor_test() / modes_run_turn_test() / other pure
 *   diagnostic modes have no reason to touch it at all.
 *
 *   STATE GRAPH
 *   ─────────────────────────────────────────────────────────────────────
 *                    ┌─────────────────────────────────────────┐
 *                    │                                          │
 *                    ▼                                          │
 *      ┌──────┐  SEARCHING  ┌───────────┐  IDLE   ┌──────────────┐
 *      │ IDLE │ ──────────► │ RETURNING │ ───────► │ IDLE (again) │
 *      └──────┘             └───────────┘          └──────────────┘
 *        │  ▲                                            │
 *        │  │ FINISHED                       SPEEDRUNNING │
 *        │  │                                             ▼
 *        │  │                                      ┌──────────────┐
 *        │  └───────────────────────────────────── │ SPEEDRUNNING │
 *        │                          IDLE            └──────────────┘
 *        │  (start a standalone speedrun,
 *        │   maze already known — modes 7/8/9)
 *        ▼
 *      IDLE ──SPEEDRUNNING──► SPEEDRUNNING ──FINISHED──► FINISHED ──IDLE──► IDLE
 *
 *      Any active state (SEARCHING / RETURNING / SPEEDRUNNING) ──► ERROR
 *      ERROR ──(after safety_clear())──► IDLE
 *
 *   In words: IDLE can start either SEARCHING (explore) or SPEEDRUNNING
 *   (maze already known). SEARCHING leads to RETURNING (drive back to
 *   start after reaching the goal), which returns to IDLE. SPEEDRUNNING
 *   leads to either IDLE (ready for the next speedrun — the Run1→Run2→
 *   Run3 sequence in AUTO_QUALIFIER) or FINISHED (the operator/mode is
 *   done). From any active state, a safety trip (see safety.h) can force
 *   an ERROR transition; ERROR only returns to IDLE once safety_clear()
 *   has been called.
 *
 *   SAFETY INTEGRATION
 *   ─────────────────────────────────────────────────────────────────────
 *   Transitioning INTO SEARCHING, RETURNING, or SPEEDRUNNING calls
 *   safety_run_start() automatically (arms the run-timeout and
 *   max-distance watchdogs from safety.c). state_machine_tick() — call
 *   this periodically from the same spin-wait loops that call
 *   scheduler_tick() — checks safety_is_tripped() while in an active
 *   state and forces an ERROR transition if it fires, so no caller has
 *   to remember to check safety state manually on every loop iteration.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Coarse-grained robot run state.
 */
typedef enum
{
    ROBOT_STATE_IDLE = 0U,     /**< Waiting — no run in progress.          */
    ROBOT_STATE_SEARCHING,     /**< Flood-fill exploration toward goal.    */
    ROBOT_STATE_RETURNING,     /**< Driving back to start after the goal.  */
    ROBOT_STATE_SPEEDRUNNING,  /**< Executing the optimised path at speed. */
    ROBOT_STATE_FINISHED,      /**< Run sequence complete.                 */
    ROBOT_STATE_ERROR          /**< Safety trip or invalid transition.     */
} RobotState_t;

/**
 * @brief  Initialise the state machine to ROBOT_STATE_IDLE.
 * @details Call once during system bring-up, after safety_init().
 * @return MM_OK always.
 */
MmResult_t state_machine_init(void);

/**
 * @brief  Attempt to transition to a new state.
 *
 * @details Validated against the state graph in the file banner above —
 *          only the transitions shown there succeed. Transitioning into
 *          SEARCHING, RETURNING, or SPEEDRUNNING calls safety_run_start()
 *          as a side effect. A transition to ROBOT_STATE_ERROR is always
 *          accepted regardless of the current state (a safety trip can
 *          happen at any time). A transition out of ROBOT_STATE_ERROR is
 *          only accepted if safety_is_tripped() is currently false (i.e.
 *          safety_clear() has already been called) — this stops a mode
 *          function from silently resuming past an unacknowledged trip.
 *
 * @param  new_state  State to transition to.
 *
 * @return MM_OK           Transition succeeded; current state updated.
 * @return MM_ERR_PARAM    new_state is not a valid RobotState_t value.
 * @return MM_ERR_GENERAL  Transition is not allowed from the current
 *                         state (see the state graph), or new_state is
 *                         ROBOT_STATE_IDLE from ROBOT_STATE_ERROR while
 *                         safety_is_tripped() is still true.
 */
MmResult_t state_machine_transition(RobotState_t new_state);

/**
 * @brief  Current robot run state.
 * @return The state most recently set by state_machine_transition(), or
 *         ROBOT_STATE_IDLE if state_machine_init() has not been called.
 */
RobotState_t state_machine_get_state(void);

/**
 * @brief  Human-readable name for a state, for logging and the OLED.
 * @param  state  State to name.
 * @return Short constant string, e.g. "SEARCHING". Never NULL — an
 *         out-of-range value returns "UNKNOWN".
 */
const char *state_machine_state_name(RobotState_t state);

/**
 * @brief  Is the robot currently mid-run?
 * @return true for SEARCHING, RETURNING, or SPEEDRUNNING; false for
 *         IDLE, FINISHED, or ERROR.
 */
bool state_machine_is_active(void);

/**
 * @brief  Poll for a safety trip during an active run and force an
 *         ERROR transition if one has occurred.
 * @details Call periodically from the same spin-wait loops that call
 *          scheduler_tick() (e.g. inside modes_run_search()'s main
 *          loop). A no-op when the current state is not one of
 *          SEARCHING/RETURNING/SPEEDRUNNING, or when no trip is latched.
 */
void state_machine_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* STATE_MACHINE_H */
