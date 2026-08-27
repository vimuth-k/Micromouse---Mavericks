/**
 * @file    floodfill.c
 * @brief   Flood-fill distance map — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "floodfill.h"
#include "maze.h"
#include "config.h"
#include <string.h>

static uint8_t s_dist[MAZE_SIZE][MAZE_SIZE];

#define QUEUE_SIZE  (MAZE_SIZE * MAZE_SIZE)

static uint16_t s_queue[QUEUE_SIZE];
static uint16_t s_q_head;
static uint16_t s_q_tail;

static inline void q_push(uint8_t row, uint8_t col)
{
    s_queue[s_q_tail] = (uint16_t)((uint16_t)row * MAZE_SIZE + col);
    s_q_tail = (uint16_t)((s_q_tail + 1U) % QUEUE_SIZE);
}

static inline void q_pop(uint8_t *row, uint8_t *col)
{
    uint16_t entry = s_queue[s_q_head];
    s_q_head = (uint16_t)((s_q_head + 1U) % QUEUE_SIZE);
    *row = (uint8_t)(entry / MAZE_SIZE);
    *col = (uint8_t)(entry % MAZE_SIZE);
}

static inline bool q_empty(void)
{
    return s_q_head == s_q_tail;
}

MmResult_t floodfill_init(void)
{
    (void)memset(s_dist, FLOOD_UNREACHABLE, sizeof(s_dist));
    s_q_head = 0U;
    s_q_tail = 0U;
    return MM_OK;
}

void floodfill_run(FloodMode_t mode)
{
    (void)memset(s_dist, FLOOD_UNREACHABLE, sizeof(s_dist));

    s_q_head = 0U;
    s_q_tail = 0U;

    if (mode == FLOOD_TO_GOAL)
    {
        for (uint8_t r = GOAL_ROW_MIN; r <= GOAL_ROW_MAX; r++)
        {
            for (uint8_t c = GOAL_COL_MIN; c <= GOAL_COL_MAX; c++)
            {
                s_dist[r][c] = FLOOD_GOAL_DIST;
                q_push(r, c);
            }
        }
    }
    else
    {
        s_dist[START_ROW][START_COL] = FLOOD_GOAL_DIST;
        q_push(START_ROW, START_COL);
    }

    while (!q_empty())
    {
        uint8_t r, c;
        q_pop(&r, &c);

        uint8_t cur_dist = s_dist[r][c];

        for (uint8_t dir = 0U; dir < NUM_DIRS; dir++)
        {
            if (!maze_can_move(r, c, dir)) { continue; }

            uint8_t nr = (uint8_t)((int8_t)r + MAZE_DR[dir]);
            uint8_t nc = (uint8_t)((int8_t)c + MAZE_DC[dir]);

            if (s_dist[nr][nc] == FLOOD_UNREACHABLE)
            {
                s_dist[nr][nc] = (uint8_t)(cur_dist + 1U);
                q_push(nr, nc);
            }
        }
    }
}

uint8_t floodfill_best_direction(uint8_t row, uint8_t col)
{
    uint8_t best_dir  = DIR_NONE;
    uint8_t best_dist = FLOOD_UNREACHABLE;

    for (uint8_t dir = 0U; dir < NUM_DIRS; dir++)
    {
        if (!maze_can_move(row, col, dir)) { continue; }

        uint8_t nr = (uint8_t)((int8_t)row + MAZE_DR[dir]);
        uint8_t nc = (uint8_t)((int8_t)col + MAZE_DC[dir]);

        uint8_t nd = s_dist[nr][nc];

        if (nd < best_dist)
        {
            best_dist = nd;
            best_dir  = dir;
        }
    }

    return best_dir;
}

uint8_t floodfill_get_distance(uint8_t row, uint8_t col)
{
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) { return FLOOD_UNREACHABLE; }
    return s_dist[row][col];
}

const uint8_t *floodfill_get_map(void)
{
    return &s_dist[0][0];
}

uint8_t floodfill_extract_path(uint8_t *buf, uint8_t buf_len)
{
    if (buf == NULL || buf_len == 0U) { return 0U; }

    uint8_t r    = START_ROW;
    uint8_t c    = START_COL;
    uint8_t step = 0U;

    if (maze_is_goal(r, c)) { return 0U; }

    while (step < buf_len)
    {
        uint8_t dir = floodfill_best_direction(r, c);

        if (dir == DIR_NONE)
        {
            break;
        }

        buf[step++] = dir;

        r = (uint8_t)((int8_t)r + MAZE_DR[dir]);
        c = (uint8_t)((int8_t)c + MAZE_DC[dir]);

        if (maze_is_goal(r, c)) { break; }
    }

    return step;
}

uint8_t floodfill_path_length(void)
{
    return s_dist[START_ROW][START_COL];
}

uint16_t floodfill_reachable_count(void)
{
    uint16_t count = 0U;
    for (uint8_t r = 0U; r < MAZE_SIZE; r++)
    {
        for (uint8_t c = 0U; c < MAZE_SIZE; c++)
        {
            if (s_dist[r][c] < FLOOD_UNREACHABLE) { count++; }
        }
    }
    return count;
}

bool floodfill_goal_reachable(void)
{
    return s_dist[START_ROW][START_COL] < FLOOD_UNREACHABLE;
}
