/**
 * @file    state_machine.c
 * @brief   MicroMaze 3 · Robot-level run state machine — implementation.
 * @details See state_machine.h for the full state graph and the
 *          rationale for why this sits above motion.c's MotionState_t
 *          and below main.c's DIP-mode dispatcher.
 *
 * @author  VDawn
 * @date    2026
 */
#include "state_machine.h"
#include "error.h"
#include "safety.h"

/* ═══════════════════════════════════════════════════════════════════════
 * Module state
 * ═══════════════════════════════════════════════════════════════════════ */

static RobotState_t s_state = ROBOT_STATE_IDLE;

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Is @p state one of the three "run in progress" states?
 */
static bool is_active_state(RobotState_t state)
{
    return (state == ROBOT_STATE_SEARCHING)
        || (state == ROBOT_STATE_RETURNING)
        || (state == ROBOT_STATE_SPEEDRUNNING);
}

/**
 * @brief  Check @p from -> @p to against the state graph documented in
 *         state_machine.h. ROBOT_STATE_ERROR is reachable from every
 *         state and is handled by the caller before this is consulted.
 */
static bool is_valid_transition(RobotState_t from, RobotState_t to)
{
    switch (from)
    {
        case ROBOT_STATE_IDLE:
            return (to == ROBOT_STATE_SEARCHING) ||
                   (to == ROBOT_STATE_SPEEDRUNNING);

        case ROBOT_STATE_SEARCHING:
            return (to == ROBOT_STATE_RETURNING);

        case ROBOT_STATE_RETURNING:
            return (to == ROBOT_STATE_IDLE);

        case ROBOT_STATE_SPEEDRUNNING:
            return (to == ROBOT_STATE_IDLE) ||
                   (to == ROBOT_STATE_FINISHED);

        case ROBOT_STATE_FINISHED:
            return (to == ROBOT_STATE_IDLE);

        case ROBOT_STATE_ERROR:
            /* Only IDLE is reachable from ERROR, and only once the trip
             * has been acknowledged — checked by the caller, since that
             * check needs safety_is_tripped(), not just the graph shape. */
            return (to == ROBOT_STATE_IDLE);

        default:
            return false;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t state_machine_init(void)
{
    s_state = ROBOT_STATE_IDLE;
    return MM_OK;
}

MmResult_t state_machine_transition(RobotState_t new_state)
{
    if (new_state > ROBOT_STATE_ERROR)
    {
        return MM_ERR_PARAM;
    }

    /* A safety trip can force ERROR from anywhere, at any time. */
    if (new_state == ROBOT_STATE_ERROR)
    {
        s_state = ROBOT_STATE_ERROR;
        return MM_OK;
    }

    /* Leaving ERROR requires the trip to have been acknowledged first —
     * otherwise a mode function could silently resume past an
     * unresolved safety condition. */
    if ((s_state == ROBOT_STATE_ERROR) && safety_is_tripped())
    {
        return MM_ERR_GENERAL;
    }

    if (!is_valid_transition(s_state, new_state))
    {
        return MM_ERR_GENERAL;
    }

    if (is_active_state(new_state))
    {
        safety_run_start();
    }

    s_state = new_state;
    return MM_OK;
}

RobotState_t state_machine_get_state(void)
{
    return s_state;
}

const char *state_machine_state_name(RobotState_t state)
{
    switch (state)
    {
        case ROBOT_STATE_IDLE:         return "IDLE";
        case ROBOT_STATE_SEARCHING:    return "SEARCHING";
        case ROBOT_STATE_RETURNING:    return "RETURNING";
        case ROBOT_STATE_SPEEDRUNNING: return "SPEEDRUNNING";
        case ROBOT_STATE_FINISHED:     return "FINISHED";
        case ROBOT_STATE_ERROR:        return "ERROR";
        default:                       return "UNKNOWN";
    }
}

bool state_machine_is_active(void)
{
    return is_active_state(s_state);
}

void state_machine_tick(void)
{
    if (is_active_state(s_state) && safety_is_tripped())
    {
        (void)state_machine_transition(ROBOT_STATE_ERROR);
    }
}
