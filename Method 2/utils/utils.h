/**
 * @file    utils.h
 * @brief   MicroMaze 3 · Small shared helpers used across every layer.
 * @details
 *   WHAT THIS MODULE PROVIDES
 *   ─────────────────────────────────────────────────────────────────────
 *   - CLAMP(x, lo, hi)   Generic clamping macro (motors.c, others).
 *   - utils_init()       One-time DWT cycle-counter setup — call before
 *                         the FIRST delay_us() call anywhere in the
 *                         firmware (main.c does this immediately after
 *                         system_clock_config(), before any peripheral
 *                         init that might need microsecond timing, e.g.
 *                         the IR emitter settle delays in ir.c).
 *   - delay_us(us)       Blocking busy-wait delay, microsecond
 *                         resolution, using the Cortex-M4 DWT cycle
 *                         counter — HAL_Delay() only offers millisecond
 *                         resolution, which is too coarse for the IR
 *                         emitter settle time (tens of microseconds).
 *   - utils_map()         Arduino-style linear re-scale of a value from
 *                         one range to another, float precision.
 *
 *   WHY DWT INSTEAD OF A TIMER
 *   ─────────────────────────────────────────────────────────────────────
 *   Every general-purpose timer on this MCU is already committed
 *   (TIM1=motor PWM, TIM2/TIM4=encoders, TIM5=1kHz control loop) —
 *   there is no spare timer to dedicate to a microsecond delay. The
 *   Cortex-M4 core already has a free-running 32-bit cycle counter
 *   (DWT->CYCCNT) built into the debug unit; enabling it costs no
 *   peripheral and no interrupt, just two register writes at boot.
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
 * @details Type-generic via macro expansion — used with int32_t in
 *          motors.c and with float elsewhere. @p x is evaluated once
 *          per branch taken (up to twice total); avoid passing an
 *          expression with side effects (e.g. x++).
 */
#define CLAMP(x, lo, hi)  (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

/**
 * @brief  Enable the Cortex-M4 DWT cycle counter for delay_us().
 *
 * @details Sets CoreDebug->DEMCR.TRCENA and DWT->CTRL.CYCCNTENA, and
 *          resets DWT->CYCCNT to 0. Must be called exactly once, early
 *          in system bring-up, before the first delay_us() call
 *          anywhere in the firmware. Calling delay_us() before this has
 *          run will busy-wait on a counter that may not be counting,
 *          producing a delay of unpredictable (possibly zero) length.
 *
 * @note   Idempotent — safe to call more than once, though main.c only
 *         calls it once, immediately after system_clock_config().
 */
void utils_init(void);

/**
 * @brief  Blocking delay with microsecond resolution.
 *
 * @details Busy-waits by polling DWT->CYCCNT against a target cycle
 *          count derived from SYSCLK_HZ. Used for hardware settle
 *          times too short for HAL_Delay()'s millisecond granularity —
 *          e.g. the IR emitter turn-on/turn-off settle delay in ir.c.
 *
 * @warning Blocking — never call from the TIM5 1 kHz control-loop ISR.
 *          Every current call site (ir.c) runs in main-loop context.
 * @warning utils_init() must have run first (see above).
 *
 * @param  us  Delay length in microseconds. 0 returns immediately.
 */
void delay_us(uint32_t us);

/**
 * @brief  Linearly re-scale @p x from [@p in_min, @p in_max] to
 *         [@p out_min, @p out_max].
 *
 * @details Arduino-style map(), float precision throughout. Does NOT
 *          clamp the result — a value of @p x outside
 *          [@p in_min, @p in_max] extrapolates linearly rather than
 *          saturating. Wrap the call in CLAMP() at the call site if
 *          saturation is required.
 *
 * @param  x        Input value to re-scale.
 * @param  in_min   Lower bound of the input range.
 * @param  in_max   Upper bound of the input range.
 * @param  out_min  Lower bound of the output range.
 * @param  out_max  Upper bound of the output range.
 *
 * @return Re-scaled value. Returns @p out_min if
 *         (@p in_max - @p in_min) is zero, to avoid a divide-by-zero.
 */
float utils_map(float x, float in_min, float in_max,
                 float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
