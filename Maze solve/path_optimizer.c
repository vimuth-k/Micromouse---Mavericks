/**
 * @file    path_optimizer.c
 * @brief   Path optimizer — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "path_optimizer.h"
#include "maze.h"
#include "config.h"
#include <string.h>

static OptPath_t s_path;

static bool append_move(MoveType_t type, uint8_t cells, float speed)
{
    if (s_path.count >= PATH_MAX_MOVES) { return false; }

    s_path.moves[s_path.count].type  = type;
    s_path.moves[s_path.count].cells = cells;
    s_path.moves[s_path.count].speed = speed;
    s_path.count++;

    return true;
}

static bool flush_straight(uint8_t run_len, float speed)
{
    if (run_len == 0U) { return true; }
    s_path.total_cells += run_len;
    return append_move(MOVE_FORWARD, run_len, speed);
}

MmResult_t path_optimizer_init(void)
{
    (void)memset(&s_path, 0, sizeof(s_path));
    return MM_OK;
}

MmResult_t path_optimizer_run(const uint8_t *dirs,
                               uint8_t        dir_count,
                               float          run_speed)
{
    if (dirs == NULL || dir_count == 0U) { return MM_ERR_PARAM; }

    (void)memset(&s_path, 0, sizeof(s_path));
    s_path.run_speed = run_speed;

    uint8_t cur_heading   = (uint8_t)START_HEADING;
    uint8_t straight_run  = 0U;

    for (uint8_t i = 0U; i < dir_count; i++)
    {
        uint8_t dir = dirs[i];

        if (dir >= NUM_DIRS) { continue; }

        if (dir == cur_heading)
        {
            straight_run++;
        }
        else
        {
            if (!flush_straight(straight_run, run_speed))
            {
                return MM_ERR_OVERFLOW;
            }

            MoveType_t turn = path_optimizer_turn_type(cur_heading, dir);
            if (turn != MOVE_DONE)
            {
                if (!append_move(turn, 0U, 0.0f))
                {
                    return MM_ERR_OVERFLOW;
                }
                s_path.total_turns++;
            }

            cur_heading  = dir;
            straight_run = 1U;
        }
    }

    if (!flush_straight(straight_run, run_speed))
    {
        return MM_ERR_OVERFLOW;
    }

    if (!append_move(MOVE_DONE, 0U, 0.0f))
    {
        return MM_ERR_OVERFLOW;
    }

    if (s_path.total_cells != dir_count)
    {
        return MM_ERR_OVERFLOW;
    }

    return MM_OK;
}

const OptPath_t *path_optimizer_get(void)
{
    return &s_path;
}

MoveType_t path_optimizer_turn_type(uint8_t from_dir, uint8_t to_dir)
{
    if (from_dir >= NUM_DIRS || to_dir >= NUM_DIRS)
    {
        return MOVE_DONE;
    }

    uint8_t diff = (uint8_t)((to_dir - from_dir + (uint8_t)NUM_DIRS)
                              % (uint8_t)NUM_DIRS);

    switch (diff)
    {
        case 1U:  return MOVE_TURN_RIGHT;
        case 3U:  return MOVE_TURN_LEFT;
        case 2U:  return MOVE_TURN_180;
        case 0U:
        default:  return MOVE_DONE;
    }
}

void path_optimizer_print(const OptPath_t *path)
{
    (void)path;
}
