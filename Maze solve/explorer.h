/**
 * @file    explorer.h
 * @brief   MicroMaze 3 · Flood-fill maze search and return-to-start.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef EXPLORER_H
#define EXPLORER_H

#include <stdint.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EXPLORER_MAX_CELLS  512U

MmResult_t explorer_search_to_goal(void);
MmResult_t explorer_return_to_start(void);

#ifdef __cplusplus
}
#endif

#endif /* EXPLORER_H */
