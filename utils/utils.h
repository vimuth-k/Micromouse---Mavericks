/**
 * @file    utils.h
 * @brief   MicroMaze 3 · Small shared helpers used across every layer.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Clamp @p x to the inclusive range [@p lo, @p hi].
 */
#define CLAMP(x, lo, hi)  (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

/**
 * @brief  Enable the Cortex-M4 DWT cycle counter for delay_us().
 */
void utils_init(void);

/**
 * @brief  Blocking delay with microsecond resolution using DWT cycle counter.
 * @param  us  Delay length in microseconds. 0 returns immediately.
 */
void delay_us(uint32_t us);

/**
 * @brief  Linearly re-scale @p x from [@p in_min, @p in_max] to [@p out_min, @p out_max].
 */
float utils_map(float x, float in_min, float in_max,
                float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
