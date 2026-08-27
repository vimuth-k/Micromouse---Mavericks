/**
 * @file    utils.c
 * @brief   MicroMaze 3 · Small shared helpers used across every layer.
 *
 * @author  VDawn
 * @date    2026
 */

#include "utils.h"
#include "config.h"   /* SYSCLK_HZ */
#include "main.h"     /* CoreDebug, DWT register definitions (CMSIS) */

static uint8_t s_dwt_ready = 0U;

void utils_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0UL;
    DWT->CTRL         |= DWT_CTRL_CYCCNTENA_Msk;

    s_dwt_ready = 1U;
}

void delay_us(uint32_t us)
{
    if (us == 0U)
    {
        return;
    }

    if (s_dwt_ready == 0U)
    {
        HAL_Delay((us + 999U) / 1000U);
        return;
    }

    uint32_t start         = DWT->CYCCNT;
    uint32_t cycles_per_us = SYSCLK_HZ / 1000000UL;
    
    /* Guard against 32-bit cycle count overflow if a huge delay is passed */
    uint32_t max_us = UINT32_MAX / cycles_per_us;
    if (us > max_us)
    {
        us = max_us;
    }

    uint32_t target_cycles = us * cycles_per_us;

    while ((DWT->CYCCNT - start) < target_cycles)
    {
        /* Busy-wait */
    }
}

float utils_map(float x, float in_min, float in_max,
                float out_min, float out_max)
{
    float in_span = in_max - in_min;

    if (in_span == 0.0f)
    {
        return out_min;
    }

    return out_min + ((x - in_min) * (out_max - out_min) / in_span);
}
