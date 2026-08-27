/**
 * @file    turn.h
 * @brief   MicroMaze 3 · Heading tracker and path/move executor.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef TURN_H
#define TURN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "error.h"
#include "path_optimizer.h"

MmResult_t turn_init(void);

void turn_execute_move(const PathMove_t *move);

MmResult_t turn_execute_path(const OptPath_t *path);

uint8_t turn_get_heading(void);

void turn_set_heading(uint8_t heading);

MmResult_t turn_run_test_sequence(void);

#ifdef __cplusplus
}
#endif

#endif /* TURN_H */
