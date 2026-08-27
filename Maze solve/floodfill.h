/**
 * @file    floodfill.h
 * @brief   Flood-fill distance map — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef FLOODFILL_H
#define FLOODFILL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "error.h"

#define FLOOD_UNREACHABLE   255U
#define FLOOD_GOAL_DIST     0U

typedef enum
{
    FLOOD_TO_GOAL  = 0U,
    FLOOD_TO_START = 1U,
} FloodMode_t;

MmResult_t floodfill_init(void);

void       floodfill_run(FloodMode_t mode);
uint8_t    floodfill_best_direction(uint8_t row, uint8_t col);
uint8_t    floodfill_get_distance(uint8_t row, uint8_t col);
const uint8_t *floodfill_get_map(void);

uint8_t    floodfill_extract_path(uint8_t *buf, uint8_t buf_len);
uint8_t    floodfill_path_length(void);

uint16_t   floodfill_reachable_count(void);
bool       floodfill_goal_reachable(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOODFILL_H */
