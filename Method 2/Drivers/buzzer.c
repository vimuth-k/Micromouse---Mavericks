/**
 * @file    buzzer.c
 * @brief   Piezo buzzer driver — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          This is the simplest driver in the firmware — and deliberately
 *          so.  The buzzer gives you audible feedback when you have no
 *          serial terminal, no OLED, and no oscilloscope.  Simplicity here
 *          means it is reliable in every context.
 *
 *          The entire module is built on two primitives:
 *            buzzer_beep(ms)               — one tone, one duration
 *            buzzer_pattern(n, on, off)    — n tones, alternating on/off
 *
 *          Every named pattern (boot, goal, error, etc.) is a single call
 *          to one of those two functions with specific timing arguments
 *          taken from config.h constants.
 *
 *          WHY BLOCKING
 *          ─────────────
 *          Buzzer events happen at controlled points:
 *            - Once at boot (before any motion starts).
 *            - Once when a button press is acknowledged.
 *            - Once when the goal is reached (robot is stationary).
 *            - Once on fatal error (motors are disabled, robot is halted).
 *            - Once when battery drops (between runs, not during motion).
 *          None of these occur during active robot motion.  The 1 kHz
 *          control loop runs from TIM5 interrupt and is unaffected by
 *          HAL_Delay() blocking in the main loop.  The buzzer never
 *          interferes with PID execution, encoder reading, or sensor updates.
 *
 *          HOW THE HARDWARE WORKS
 *          ───────────────────────
 *          PB1 → 100 Ω → base of NPN transistor → buzzer → 3.3 V
 *          When PB1 = HIGH: transistor conducts, current flows, buzzer sounds.
 *          When PB1 = LOW:  transistor cuts off, no current, silence.
 *
 *          For an active buzzer (self-oscillating, most common):
 *            Typical current draw: 25–30 mA at 3.3 V.
 *            STM32 GPIO sink/source limit: 25 mA per pin.
 *            ∴ A transistor is REQUIRED to avoid damaging the STM32 GPIO.
 *            The 100 Ω base resistor limits base current to ~33 mA ÷ β.
 *
 * @author  VDawn
 * @date    2026
 */

#include "buzzer.h"
#include "pins.h"
#include "config.h"
#include "stm32f4xx_hal.h"

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the buzzer — ensure pin starts LOW (silent).
 */
MmResult_t buzzer_init(void)
{
    BUZZER_OFF();
    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — PRIMITIVES
 * ======================================================================= */

/**
 * @brief  Single blocking beep.
 */
void buzzer_beep(uint32_t ms)
{
    if (ms == 0U) { return; }
    BUZZER_ON();
    HAL_Delay(ms);
    BUZZER_OFF();
}

/**
 * @brief  Repeated blocking beep pattern.
 *
 * @details Produces 'count' beeps.  A gap of 'off_ms' is inserted
 *          between beeps but NOT after the last one — the caller
 *          controls any post-pattern silence.
 */
void buzzer_pattern(uint8_t count, uint32_t on_ms, uint32_t off_ms)
{
    if (count == 0U) { return; }

    for (uint8_t i = 0U; i < count; i++)
    {
        BUZZER_ON();
        HAL_Delay(on_ms);
        BUZZER_OFF();

        /* Gap between beeps — not after the last one */
        if (i < (count - 1U))
        {
            HAL_Delay(off_ms);
        }
    }
}

/* =========================================================================
 * PUBLIC API — NAMED PATTERNS
 * ======================================================================= */

/**
 * @brief  Boot confirmation — 2 short beeps.
 *         ON(100ms) gap(80ms) ON(100ms) — total ~280 ms
 */
void buzzer_boot(void)
{
    buzzer_pattern(2U, BUZZ_BOOT_MS, BUZZ_BOOT_GAP_MS);
}

/**
 * @brief  Run start — 1 short beep.
 *         ON(50ms) — total 50 ms
 */
void buzzer_run_start(void)
{
    buzzer_beep(BUZZ_START_MS);
}

/**
 * @brief  Goal reached — 1 long beep.
 *         ON(500ms) — total 500 ms
 */
void buzzer_goal(void)
{
    buzzer_beep(BUZZ_GOAL_MS);
}

/**
 * @brief  Fatal error — 3 long beeps.
 *         ON(300ms) gap(150ms) × 3 — total ~1050 ms
 */
void buzzer_error(void)
{
    buzzer_pattern(3U, BUZZ_ERROR_MS, BUZZ_ERROR_GAP_MS);
}

/**
 * @brief  Low battery warning — 3 rapid beeps.
 *         ON(60ms) gap(60ms) × 3 — total ~360 ms
 */
void buzzer_low_battery(void)
{
    buzzer_pattern(3U, BUZZ_LOWBATT_MS, BUZZ_LOWBATT_GAP_MS);
}

/**
 * @brief  Search run complete — 2 medium beeps.
 *         ON(150ms) gap(100ms) ON(150ms) — total ~400 ms
 */
void buzzer_search_done(void)
{
    buzzer_pattern(2U, 150U, 100U);
}

/**
 * @brief  Calibration saved — 3 ascending-length beeps.
 *
 * @details Each beep is longer than the last, giving an audible
 *          "rising" confirmation distinct from the error pattern.
 *          ON(80ms) gap(60ms) ON(120ms) gap(60ms) ON(200ms)
 *          Total: ~520 ms
 */
void buzzer_calibrated(void)
{
    buzzer_beep(80U);
    HAL_Delay(60U);
    buzzer_beep(120U);
    HAL_Delay(60U);
    buzzer_beep(200U);
}

/* =========================================================================
 * PUBLIC API — UTILITY
 * ======================================================================= */

/**
 * @brief  Immediately silence the buzzer.
 */
void buzzer_off(void)
{
    BUZZER_OFF();
}
