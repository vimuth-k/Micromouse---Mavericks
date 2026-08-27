/**
 * @file    speedrun.h
 * @brief   MicroMaze 3 · Execute the optimised path at speed.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef SPEEDRUN_H
#define SPEEDRUN_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

MmResult_t speedrun_run(float speed_mmps);

#ifdef __cplusplus
}
#endif

#endif /* SPEEDRUN_H */
