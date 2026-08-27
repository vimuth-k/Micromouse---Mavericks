/**
 * @file    path_optimizer.h
 * @brief   Path optimizer — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "error.h"

#define PATH_MAX_MOVES  128U

typedef enum
{
    MOVE_FORWARD    = 0U,
    MOVE_TURN_RIGHT = 1U,
    MOVE_TURN_LEFT  = 2U,
    MOVE_TURN_180   = 3U,
    MOVE_DONE       = 4U,
} MoveType_t;

typedef struct
{
    MoveType_t type;
    uint8_t    cells;
    float      speed;
} PathMove_t;

typedef struct
{
    PathMove_t moves[PATH_MAX_MOVES];
    uint8_t    count;
    uint8_t    total_cells;
    uint8_t    total_turns;
    float      run_speed;
} OptPath_t;

MmResult_t path_optimizer_init(void);

MmResult_t path_optimizer_run(const uint8_t *dirs,
                              uint8_t        dir_count,
                              float          run_speed);

const OptPath_t *path_optimizer_get(void);

MoveType_t path_optimizer_turn_type(uint8_t from_dir, uint8_t to_dir);

void path_optimizer_print(const OptPath_t *path);

#ifdef __cplusplus
}
#endif

#endif /* PATH_OPTIMIZER_H */
