/**
 * @file    state_machine.h
 * @brief   MicroMaze 3 · Robot-level run state machine.
 * @details Coordinates high-level run states and safety arming during execution.
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
 * @brief  Attempt to transition to a new state.
 * @details Enforces valid state flow and arms safety watchdogs on run start.
 * @param  new_state  State to transition to.
 * @return MM_OK on success, MM_ERR_PARAM or MM_ERR_GENERAL on invalid transition.
 */
MmResult_t state_machine_transition(RobotState_t new_state);

#ifdef __cplusplus
}
#endif

#endif /* STATE_MACHINE_H */
