/**
 * @file    explorer.c
 * @brief   MicroMaze 3 · Flood-fill maze search and return-to-start —
 *          implementation.
 * @details See explorer.h for the full design rationale, including why
 *          the search and return phases are kept as separate functions
 *          and why every cell re-runs floodfill_run() unconditionally.
 *
 * @author  VDawn
 * @date    2026
 */
#include "explorer.h"
#include "config.h"
#include "error.h"
#include "logger.h"
#include "motion.h"
#include "maze.h"
#include "floodfill.h"
#include "path_optimizer.h"  /* path_optimizer_turn_type(), MoveType_t */
#include "safety.h"
#include "buzzer.h"

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Turn (if needed) to face @p dir, then advance one cell,
 *         keeping the physical robot (motion.c) and the tracked
 *         position/heading (maze.c) in sync.
 */
static void explorer_step_toward(uint8_t dir)
{
    MoveType_t turn = path_optimizer_turn_type(maze.robot_heading, dir);

    switch (turn)
    {
        case MOVE_TURN_RIGHT:
            motion_align_front();
            motion_turn_right();
            maze_turn_right();
            break;

        case MOVE_TURN_LEFT:
            motion_align_front();
            motion_turn_left();
            maze_turn_left();
            break;

        case MOVE_TURN_180:
            motion_align_front();
            motion_turn_180();
            maze_turn_180();
            break;

        case MOVE_DONE:
        case MOVE_FORWARD:
        default:
            /* Already facing dir — no turn needed. */
            break;
    }

    move_forward(1U, SPD_SEARCH);
    maze_advance(dir);
}

/**
 * @brief  Shared per-cell loop body for both phases: record walls,
 *         re-flood, check the target, pick a direction, and step.
 *
 * @param  mode        Flood-fill seeding mode for this phase.
 * @param  at_target    Returns true when the current cell is where this
 *                      phase is trying to reach.
 * @param  phase_name  Short label for log messages ("search"/"return").
 *
 * @return MM_OK           at_target() became true.
 * @return MM_ERR_GENERAL  Cell limit reached, no valid move, or a
 *                        safety trip aborted the loop.
 */
static MmResult_t explorer_run_loop(FloodMode_t mode, bool (*at_target)(void),
                                     const char *phase_name)
{
    for (uint16_t cell_count = 0U; cell_count < EXPLORER_MAX_CELLS; cell_count++)
    {
        maze_update_walls_from_sensors(maze.robot_row, maze.robot_col, maze.robot_heading);
        floodfill_run(mode);

        if (at_target())
        {
            LOG_INFO("Explorer: %s reached target in %u cells", phase_name, (unsigned)cell_count);
            return MM_OK;
        }

        if (safety_is_tripped())
        {
            LOG_ERROR("Explorer: %s aborted at cell %u — safety trip (reason %d)",
                      phase_name, (unsigned)cell_count, (int)safety_trip_reason());
            return MM_ERR_GENERAL;
        }

        uint8_t dir = floodfill_best_direction(maze.robot_row, maze.robot_col);
        if (dir == DIR_NONE)
        {
            LOG_ERROR("Explorer: %s — no valid move from (%u,%u), maze inconsistency",
                      phase_name, maze.robot_row, maze.robot_col);
            return MM_ERR_GENERAL;
        }

        explorer_step_toward(dir);
    }

    LOG_WARN("Explorer: %s — cell limit (%u) reached without success",
             phase_name, (unsigned)EXPLORER_MAX_CELLS);
    return MM_ERR_GENERAL;
}

static bool at_goal(void)
{
    return maze_is_goal(maze.robot_row, maze.robot_col);
}

static bool at_start(void)
{
    return (maze.robot_row == START_ROW) && (maze.robot_col == START_COL);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t explorer_init(void)
{
    return MM_OK;
}

MmResult_t explorer_search_to_goal(void)
{
    LOG_INFO("Explorer: search phase starting from (%u,%u)",
             maze.robot_row, maze.robot_col);

    MmResult_t result = explorer_run_loop(FLOOD_TO_GOAL, at_goal, "search");

    if (result == MM_OK)
    {
        buzzer_goal();

        /* Persist now — enough map data exists to compute at least one
         * path to the goal, and a standalone speed-run mode selected
         * after a reboot needs this even if the operator never runs
         * explorer_return_to_start() in the same session. */
        if (maze_save_to_flash() != MM_OK)
        {
            LOG_WARN("Explorer: search succeeded but Flash save failed — "
                     "map will not survive a reboot");
        }
    }

    return result;
}

MmResult_t explorer_return_to_start(void)
{
    LOG_INFO("Explorer: return phase starting from (%u,%u)",
             maze.robot_row, maze.robot_col);

    MmResult_t result = explorer_run_loop(FLOOD_TO_START, at_start, "return");

    if (result == MM_OK)
    {
        /* More wall data has typically accumulated on the way back —
         * save again so speed runs benefit from the fuller map. */
        if (maze_save_to_flash() != MM_OK)
        {
            LOG_WARN("Explorer: return succeeded but Flash save failed — "
                     "map will not survive a reboot");
        }
    }

    return result;
}
