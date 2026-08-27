/**
 * @file    wall_follow.c
 * @brief   MicroMaze 3 · Left-hand-rule fallback maze solver — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "wall_follow.h"
#include "config.h"
#include "error.h"
#include "motion.h"
#include "maze.h"
#include "ir.h"
#include "safety.h"

static void wall_follow_step(void)
{
    maze_update_walls_from_sensors(maze.robot_row, maze.robot_col, maze.robot_heading);

    bool left_clear  = !ir_wall_left();
    bool front_clear = !ir_wall_front();
    bool right_clear = !ir_wall_right();

    if (left_clear)
    {
        motion_align_front();
        motion_turn_left();
        maze_turn_left();
    }
    else if (front_clear)
    {
        /* Straight ahead */
    }
    else if (right_clear)
    {
        motion_align_front();
        motion_turn_right();
        maze_turn_right();
    }
    else
    {
        motion_align_front();
        motion_turn_180();
        maze_turn_180();
    }

    move_forward(1U, SPD_SEARCH);
    maze_advance(maze.robot_heading);
}

MmResult_t wall_follow_run(void)
{
    for (uint16_t cell_count = 0U; cell_count < WALL_FOLLOW_MAX_CELLS; cell_count++)
    {
        if (maze_is_goal(maze.robot_row, maze.robot_col))
        {
            return MM_OK;
        }

        if (safety_is_tripped())
        {
            return MM_ERR_GENERAL;
        }

        wall_follow_step();
    }

    return MM_ERR_GENERAL;
}
