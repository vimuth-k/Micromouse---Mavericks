/**
 * @file    maze.c
 * @brief   Maze data structures and wall map — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "maze.h"
#include "ir.h"
#include "flash_storage.h"
#include "config.h"
#include "main.h"
#include <string.h>

MazeState_t maze;

const int8_t MAZE_DR[NUM_DIRS] = { -1,  0,  1,  0 };
const int8_t MAZE_DC[NUM_DIRS] = {  0,  1,  0, -1 };

const uint8_t MAZE_WALL_FROM[NUM_DIRS] = { WALL_N, WALL_E, WALL_S, WALL_W };
const uint8_t MAZE_WALL_OPP[NUM_DIRS]  = { WALL_S, WALL_W, WALL_N, WALL_E };

static bool in_bounds(uint8_t row, uint8_t col)
{
    return (row < MAZE_SIZE) && (col < MAZE_SIZE);
}

static void set_boundary_walls(void)
{
    for (uint8_t i = 0U; i < MAZE_SIZE; i++)
    {
        maze.walls[0U][i]              |= WALL_N;
        maze.walls[MAZE_SIZE - 1U][i]  |= WALL_S;
        maze.walls[i][0U]              |= WALL_W;
        maze.walls[i][MAZE_SIZE - 1U]  |= WALL_E;
    }
}

MmResult_t maze_init(void)
{
    (void)memset(maze.walls,   0, sizeof(maze.walls));
    (void)memset(maze.visited, 0, sizeof(maze.visited));

    set_boundary_walls();

    maze.walls[START_ROW][START_COL] |= WALL_E;

    maze.robot_row     = START_ROW;
    maze.robot_col     = START_COL;
    maze.robot_heading = START_HEADING;

    return MM_OK;
}

MmResult_t maze_clear_walls(void)
{
    (void)memset(maze.walls,   0, sizeof(maze.walls));
    (void)memset(maze.visited, 0, sizeof(maze.visited));
    set_boundary_walls();
    maze.walls[START_ROW][START_COL] |= WALL_E;
    return MM_OK;
}

bool maze_has_wall(uint8_t row, uint8_t col, uint8_t dir)
{
    if (!in_bounds(row, col) || dir >= NUM_DIRS) { return true; }
    return (maze.walls[row][col] & MAZE_WALL_FROM[dir]) != 0U;
}

bool maze_can_move(uint8_t row, uint8_t col, uint8_t dir)
{
    if (!in_bounds(row, col) || dir >= NUM_DIRS) { return false; }
    if (maze.walls[row][col] & MAZE_WALL_FROM[dir]) { return false; }

    int8_t nr = (int8_t)row + MAZE_DR[dir];
    int8_t nc = (int8_t)col + MAZE_DC[dir];
    return in_bounds((uint8_t)nr, (uint8_t)nc);
}

uint8_t maze_get_walls(uint8_t row, uint8_t col)
{
    if (!in_bounds(row, col)) { return 0xFFU; }
    return maze.walls[row][col];
}

bool maze_is_visited(uint8_t row, uint8_t col)
{
    if (!in_bounds(row, col)) { return false; }
    return maze.visited[row][col] != 0U;
}

bool maze_is_goal(uint8_t row, uint8_t col)
{
    return (row >= GOAL_ROW_MIN) && (row <= GOAL_ROW_MAX) &&
           (col >= GOAL_COL_MIN) && (col <= GOAL_COL_MAX);
}

bool maze_is_start(uint8_t row, uint8_t col)
{
    return (row == START_ROW) && (col == START_COL);
}

void maze_set_wall(uint8_t row, uint8_t col, uint8_t dir)
{
    if (!in_bounds(row, col) || dir >= NUM_DIRS) { return; }

    maze.walls[row][col] |= MAZE_WALL_FROM[dir];

    int8_t nr = (int8_t)row + MAZE_DR[dir];
    int8_t nc = (int8_t)col + MAZE_DC[dir];

    if (in_bounds((uint8_t)nr, (uint8_t)nc))
    {
        maze.walls[(uint8_t)nr][(uint8_t)nc] |= MAZE_WALL_OPP[dir];
    }
}

void maze_update_walls_from_sensors(uint8_t row, uint8_t col, uint8_t heading)
{
    if (!in_bounds(row, col) || heading >= NUM_DIRS) { return; }

    uint8_t dir_front = heading;
    uint8_t dir_left  = maze_relative_to_absolute(heading, 3U);
    uint8_t dir_right = maze_relative_to_absolute(heading, 1U);

    if (ir_wall_front())  { maze_set_wall(row, col, dir_front); }
    if (ir_wall_left())   { maze_set_wall(row, col, dir_left);  }
    if (ir_wall_right())  { maze_set_wall(row, col, dir_right); }

    maze.visited[row][col] = 1U;
}

void maze_advance(uint8_t dir)
{
    if (dir >= NUM_DIRS) { return; }
    maze.robot_row     = (uint8_t)((int8_t)maze.robot_row + MAZE_DR[dir]);
    maze.robot_col     = (uint8_t)((int8_t)maze.robot_col + MAZE_DC[dir]);
    maze.robot_heading = dir;
}

void maze_turn_right(void)
{
    maze.robot_heading = (maze.robot_heading + 1U) % NUM_DIRS;
}

void maze_turn_left(void)
{
    maze.robot_heading = (maze.robot_heading + 3U) % NUM_DIRS;
}

void maze_turn_180(void)
{
    maze.robot_heading = (maze.robot_heading + 2U) % NUM_DIRS;
}

void maze_set_position(uint8_t row, uint8_t col, uint8_t heading)
{
    maze.robot_row     = in_bounds(row, col) ? row    : START_ROW;
    maze.robot_col     = in_bounds(row, col) ? col    : START_COL;
    maze.robot_heading = (heading < NUM_DIRS)? heading: DIR_N;
}

uint8_t maze_relative_to_absolute(uint8_t heading, uint8_t relative)
{
    return (uint8_t)((heading + relative) % NUM_DIRS);
}

uint8_t maze_opposite_dir(uint8_t dir)
{
    return (uint8_t)((dir + 2U) % NUM_DIRS);
}

void maze_neighbour(uint8_t row, uint8_t col, uint8_t dir,
                    uint8_t *nr, uint8_t *nc)
{
    if (nr != NULL) { *nr = (uint8_t)((int8_t)row + MAZE_DR[dir]); }
    if (nc != NULL) { *nc = (uint8_t)((int8_t)col + MAZE_DC[dir]); }
}

char maze_dir_char(uint8_t dir)
{
    static const char CHARS[NUM_DIRS] = { 'N', 'E', 'S', 'W' };
    return (dir < NUM_DIRS) ? CHARS[dir] : '?';
}

uint16_t maze_visited_count(void)
{
    uint16_t count = 0U;
    for (uint8_t r = 0U; r < MAZE_SIZE; r++)
    {
        for (uint8_t c = 0U; c < MAZE_SIZE; c++)
        {
            if (maze.visited[r][c]) { count++; }
        }
    }
    return count;
}

bool maze_is_fully_explored(void)
{
    for (uint8_t r = 0U; r < MAZE_SIZE; r++)
    {
        for (uint8_t c = 0U; c < MAZE_SIZE; c++)
        {
            if (!maze.visited[r][c] && maze.walls[r][c] != 0xFFU)
            {
                return false;
            }
        }
    }
    return true;
}

MmResult_t maze_save_to_flash(void)
{
    HAL_StatusTypeDef st;
    uint32_t addr = FLASH_MAZE_ADDR;

    if (flash_storage_prepare_write(FLASH_REGION_MAZE) != MM_OK)
    {
        return MM_ERR_STORAGE;
    }

    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, FLASH_MAZE_MAGIC);
    if (st != HAL_OK) { (void)HAL_FLASH_Lock(); return MM_ERR_STORAGE; }
    addr += 4U;

    const uint32_t *walls_u32 = (const uint32_t *)maze.walls;
    uint32_t walls_words = (sizeof(maze.walls) + 3U) / 4U;
    for (uint32_t w = 0U; w < walls_words; w++)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, walls_u32[w]);
        if (st != HAL_OK) { (void)HAL_FLASH_Lock(); return MM_ERR_STORAGE; }
        addr += 4U;
    }

    const uint32_t *vis_u32  = (const uint32_t *)maze.visited;
    uint32_t vis_words = (sizeof(maze.visited) + 3U) / 4U;
    for (uint32_t w = 0U; w < vis_words; w++)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, vis_u32[w]);
        if (st != HAL_OK) { (void)HAL_FLASH_Lock(); return MM_ERR_STORAGE; }
        addr += 4U;
    }

    (void)HAL_FLASH_Lock();
    return MM_OK;
}

MmResult_t maze_load_from_flash(void)
{
    uint32_t magic = *(const uint32_t *)FLASH_MAZE_ADDR;
    if (magic != FLASH_MAZE_MAGIC) { return MM_ERR_NOT_FOUND; }

    const uint8_t *src = (const uint8_t *)(FLASH_MAZE_ADDR + 4U);
    (void)memcpy(maze.walls,   src,                       sizeof(maze.walls));
    (void)memcpy(maze.visited, src + sizeof(maze.walls),  sizeof(maze.visited));

    return MM_OK;
}

void maze_print(const uint8_t flood[MAZE_SIZE][MAZE_SIZE])
{
    (void)flood;
}
