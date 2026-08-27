/**
 * @file    maze.h
 * @brief   Maze data structures and wall map — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef MAZE_H
#define MAZE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"
#include "config.h"

#define DIR_N       0U
#define DIR_E       1U
#define DIR_S       2U
#define DIR_W       3U
#define DIR_NONE    0xFFU
#define NUM_DIRS    4U

#define WALL_N      (1U << DIR_N)
#define WALL_E      (1U << DIR_E)
#define WALL_S      (1U << DIR_S)
#define WALL_W      (1U << DIR_W)

typedef struct
{
    uint8_t walls[MAZE_SIZE][MAZE_SIZE];
    uint8_t visited[MAZE_SIZE][MAZE_SIZE];
    uint8_t robot_row;
    uint8_t robot_col;
    uint8_t robot_heading;
} MazeState_t;

extern MazeState_t maze;

extern const int8_t  MAZE_DR[NUM_DIRS];
extern const int8_t  MAZE_DC[NUM_DIRS];
extern const uint8_t MAZE_WALL_FROM[NUM_DIRS];
extern const uint8_t MAZE_WALL_OPP[NUM_DIRS];

MmResult_t maze_init(void);
MmResult_t maze_clear_walls(void);

bool       maze_has_wall(uint8_t row, uint8_t col, uint8_t dir);
bool       maze_can_move(uint8_t row, uint8_t col, uint8_t dir);
uint8_t    maze_get_walls(uint8_t row, uint8_t col);
bool       maze_is_visited(uint8_t row, uint8_t col);
bool       maze_is_goal(uint8_t row, uint8_t col);
bool       maze_is_start(uint8_t row, uint8_t col);

void       maze_set_wall(uint8_t row, uint8_t col, uint8_t dir);
void       maze_update_walls_from_sensors(uint8_t row, uint8_t col, uint8_t heading);

void       maze_advance(uint8_t dir);
void       maze_turn_right(void);
void       maze_turn_left(void);
void       maze_turn_180(void);
void       maze_set_position(uint8_t row, uint8_t col, uint8_t heading);

uint8_t    maze_relative_to_absolute(uint8_t heading, uint8_t relative);
uint8_t    maze_opposite_dir(uint8_t dir);
void       maze_neighbour(uint8_t row, uint8_t col, uint8_t dir,
                          uint8_t *nr, uint8_t *nc);
char       maze_dir_char(uint8_t dir);

uint16_t   maze_visited_count(void);
bool       maze_is_fully_explored(void);

MmResult_t maze_save_to_flash(void);
MmResult_t maze_load_from_flash(void);

void       maze_print(const uint8_t flood[MAZE_SIZE][MAZE_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_H */
