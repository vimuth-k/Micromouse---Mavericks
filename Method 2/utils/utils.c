/**
 * @file    utils.c
 * @brief   MicroMaze 3 · Small shared helpers used across every layer.
 * @details See utils.h for the design rationale (DWT cycle counter vs
 *          a dedicated timer — every general-purpose timer on this MCU
 *          is already committed to motors, encoders, or the control
 *          loop).
 *
 * @author  VDawn
 * @date    2026
 */
#include "utils.h"
#include "config.h"   /* SYSCLK_HZ */
#include "main.h"     /* CoreDebug, DWT register definitions (CMSIS) */

/* ═══════════════════════════════════════════════════════════════════════
 * Module state
 * ═══════════════════════════════════════════════════════════════════════ */

/** Set once utils_init() has enabled the DWT cycle counter. */
static uint8_t s_dwt_ready = 0U;

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

void utils_init(void)
{
    /* Unlock trace/debug block access, then enable the free-running
     * cycle counter. Both registers live in the Cortex-M4 core debug
     * peripherals — no clock gating, no NVIC entry required. */
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
        /* utils_init() was never called — DWT is not guaranteed to be
         * counting. Fail safe rather than spin forever or return
         * immediately and silently violate the caller's timing
         * requirement: fall back to the coarser millisecond delay so
         * the settle time is still *at least* as long as requested. */
        HAL_Delay((us + 999U) / 1000U);
        return;
    }

    /* SYSCLK_HZ is 100 MHz -> 100 cycles per microsecond. Cast to
     * uint64_t for the multiply so this stays correct even if us is
     * large enough that the intermediate would overflow a 32-bit
     * value (SYSCLK_HZ / 1e6 * us). */
    uint32_t start        = DWT->CYCCNT;
    uint32_t cycles_per_us = SYSCLK_HZ / 1000000UL;
    uint32_t target_cycles = (uint32_t)((uint64_t)us * cycles_per_us);

    /* DWT->CYCCNT is a free-running 32-bit counter that wraps around;
     * unsigned subtraction handles the wraparound correctly regardless
     * of whether CYCCNT has rolled over since `start` was captured. */
    while ((DWT->CYCCNT - start) < target_cycles)
    {
        /* Busy-wait. */
    }
}

float utils_map(float x, float in_min, float in_max,
                 float out_min, float out_max)
{
    float in_span = in_max - in_min;

    if (in_span == 0.0f)
    {
        return out_min; /* Degenerate input range — avoid divide-by-zero. */
    }

    return out_min + ((x - in_min) * (out_max - out_min) / in_span);
}
