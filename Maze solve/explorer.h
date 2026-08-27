/**
 * @file    explorer.h
 * @brief   MicroMaze 3 · Flood-fill maze search and return-to-start (DIP Mode 6).
 * @details Drives BFS-guided exploration to the goal and back to the start cell
 *          using floodfill and motion control.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef EXPLORER_H
#define EXPLORER_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Flood-fill search from the robot's current tracked position
 *         until any goal cell is reached.
 *
 * @warning Main-loop context only. Blocks until the goal is reached,
 *          EXPLORER_MAX_CELLS is hit, or a safety trip aborts the run.
 *
 * @return MM_OK           Goal reached.
 * @return MM_ERR_GENERAL  Aborted (cell limit exceeded, invalid move, or safety trip).
 */
MmResult_t explorer_search_to_goal(void);

/**
 * @brief  Flood-fill navigation from the robot's current tracked
 *         position back to the start cell.
 *
 * @warning Main-loop context only. Blocks until the start cell is reached,
 *          EXPLORER_MAX_CELLS is hit, or a safety trip aborts the run.
 *
 * @return MM_OK           Back at the start cell.
 * @return MM_ERR_GENERAL  Aborted (cell limit exceeded, invalid move, or safety trip).
 */
MmResult_t explorer_return_to_start(void);

#ifdef __cplusplus
}
#endif

#endif /* EXPLORER_H */
