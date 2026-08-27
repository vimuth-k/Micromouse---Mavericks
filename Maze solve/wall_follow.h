/**
 * @file    wall_follow.h
 * @brief   MicroMaze 3 · Left-hand-rule fallback maze solver.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <stdint.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WALL_FOLLOW_MAX_CELLS  512U

MmResult_t wall_follow_run(void);

#ifdef __cplusplus
}
#endif

#endif /* WALL_FOLLOW_H */
