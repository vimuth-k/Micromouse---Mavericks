/**
 * @file    wall_follow.h
 * @brief   MicroMaze 3 · Left-hand-rule maze solver (DIP Mode 5).
 * @details Reactive cell-by-cell left-hand-rule maze traversal using IR sensors.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Run the left-hand-rule solver from the robot's current
 *         tracked position until the goal is reached or
 *         WALL_FOLLOW_MAX_CELLS is exceeded.
 *
 * @warning Main-loop context only. Blocks until the goal is reached,
 *          the cell limit is hit, or a safety trip aborts the run.
 *
 * @return MM_OK           Goal reached.
 * @return MM_ERR_GENERAL  Aborted (cell limit exceeded or safety trip).
 */
MmResult_t wall_follow_run(void);

#ifdef __cplusplus
}
#endif

#endif /* WALL_FOLLOW_H */
