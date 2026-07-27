/**
 * @file    wall_follow.c
 * @brief   MicroMaze 3 · Left-hand-rule fallback maze solver — implementation.
 * @details See wall_follow.h for the full design rationale, including
 *          why this uses maze.c's position tracker instead of turn.c's.
 *
 * @author  VDawn
 * @date    2026
 */
#include "wall_follow.h"
#include "config.h"
#include "error.h"
#include "logger.h"
#include "motion.h"
#include "maze.h"
#include "ir.h"
#include "safety.h"
#include "buzzer.h"

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Sense, decide, and execute one left-hand-rule step, advancing
 *         exactly one cell.
 */
static void wall_follow_step(void)
{
    /* Record whatever this cell's sensors see, regardless of which way
     * the algorithm ends up going — free map data for later. */
    maze_update_walls_from_sensors(maze.robot_row, maze.robot_col, maze.robot_heading);

    bool left_clear  = !ir_wall_left();
    bool front_clear = !ir_wall_front();
    bool right_clear = !ir_wall_right();

    if (left_clear)
    {
        motion_align_front();   /* No-op if there's no front wall to square against. */
        motion_turn_left();
        maze_turn_left();
    }
    else if (front_clear)
    {
        /* Straight ahead — no turn needed. */
    }
    else if (right_clear)
    {
        motion_align_front();
        motion_turn_right();
        maze_turn_right();
    }
    else
    {
        /* Dead end on all three sides — turn back. */
        motion_align_front();
        motion_turn_180();
        maze_turn_180();
    }

    move_forward(1U, SPD_SEARCH);
    maze_advance(maze.robot_heading);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t wall_follow_init(void)
{
    return MM_OK;
}

MmResult_t wall_follow_run(void)
{
    LOG_INFO("Wall follower: starting from (%u,%u) heading %u",
             maze.robot_row, maze.robot_col, maze.robot_heading);

    for (uint16_t cell_count = 0U; cell_count < WALL_FOLLOW_MAX_CELLS; cell_count++)
    {
        if (maze_is_goal(maze.robot_row, maze.robot_col))
        {
            LOG_INFO("Wall follower: goal reached in %u cells", (unsigned)cell_count);
            buzzer_goal();
            return MM_OK;
        }

        if (safety_is_tripped())
        {
            LOG_ERROR("Wall follower: aborting at cell %u — safety trip (reason %d)",
                      (unsigned)cell_count, (int)safety_trip_reason());
            return MM_ERR_GENERAL;
        }

        wall_follow_step();
    }

    LOG_WARN("Wall follower: cell limit (%u) reached without finding goal",
             (unsigned)WALL_FOLLOW_MAX_CELLS);
    return MM_ERR_GENERAL;
}
