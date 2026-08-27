/**
 * @file    turn.c
 * @brief   MicroMaze 3 · Heading tracker and path/move executor — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "turn.h"
#include "config.h"
#include "error.h"
#include "motion.h"
#include "maze.h"
#include "safety.h"
#include "oled.h"

static uint8_t s_heading = DIR_N;

MmResult_t turn_init(void)
{
    s_heading = DIR_N;
    return MM_OK;
}

void turn_execute_move(const PathMove_t *move)
{
    if (move == NULL)
    {
        return;
    }

    switch (move->type)
    {
        case MOVE_FORWARD:
            move_forward(move->cells, move->speed);
            break;

        case MOVE_TURN_RIGHT:
            motion_align_front();
            motion_turn_right();
            s_heading = (uint8_t)((s_heading + 1U) % 4U);
            break;

        case MOVE_TURN_LEFT:
            motion_align_front();
            motion_turn_left();
            s_heading = (uint8_t)((s_heading + 3U) % 4U);
            break;

        case MOVE_TURN_180:
            motion_align_front();
            motion_turn_180();
            s_heading = (uint8_t)((s_heading + 2U) % 4U);
            break;

        case MOVE_DONE:
        default:
            break;
    }
}

MmResult_t turn_execute_path(const OptPath_t *path)
{
    if (path == NULL)
    {
        return MM_ERR_PARAM;
    }

    for (uint16_t i = 0U; i < path->count; i++)
    {
        const PathMove_t *move = &path->moves[i];

        if (move->type == MOVE_DONE)
        {
            break;
        }

        turn_execute_move(move);

        if (safety_is_tripped())
        {
            return MM_ERR_GENERAL;
        }
    }

    return MM_OK;
}

uint8_t turn_get_heading(void)
{
    return s_heading;
}

void turn_set_heading(uint8_t heading)
{
    if (heading <= DIR_W)
    {
        s_heading = heading;
    }
}

MmResult_t turn_run_test_sequence(void)
{
    PathMove_t step;
    step.cells = 0U;
    step.speed = 0.0f;

    oled_show_message("TURN TEST", "90 RIGHT");
    step.type = MOVE_TURN_RIGHT;
    turn_execute_move(&step);

    oled_show_message("TURN TEST", "180");
    step.type = MOVE_TURN_180;
    turn_execute_move(&step);

    oled_show_message("TURN TEST", "180 (360 TOTAL)");
    step.type = MOVE_TURN_180;
    turn_execute_move(&step);

    oled_show_message("TURN TEST", "DONE - CHECK");

    return MM_OK;
}
