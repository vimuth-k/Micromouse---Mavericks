/**
 * @file    buzzer.h
 * @brief   Piezo buzzer driver — public API.
 *
 * @details Provides blocking and non-blocking beep patterns for the
 *          passive piezo buzzer driven from PB1 via a small NPN
 *          transistor or N-MOSFET.
 *
 *          HARDWARE
 *          ────────
 *          Buzzer : Passive piezo element (resonant frequency ~2–4 kHz).
 *          Driver : NPN transistor (e.g. 2N2222) or N-MOSFET (e.g. AO3400A).
 *                   Base/gate resistor: 100 Ω from PB1.
 *                   Flyback diode across buzzer (if using DC buzzer).
 *          Pin    : PB1 (BUZZER_ON/OFF/TOGGLE macros in pins.h).
 *          Logic  : Active HIGH — PB1 HIGH = buzzer on.
 *
 *          PASSIVE vs ACTIVE BUZZER
 *          ─────────────────────────
 *          This driver is written for an ACTIVE buzzer (built-in
 *          oscillator) driven by DC.  Toggling the pin HIGH = sound ON,
 *          LOW = sound OFF.  The tone frequency is fixed by the buzzer
 *          itself — typically 2.3–4 kHz.
 *
 *          If you use a PASSIVE piezo (no internal oscillator), replace
 *          the BUZZER_ON/OFF calls with PWM output on PB1 (TIM3_CH4
 *          alternate function) and configure the PWM frequency to your
 *          target tone.  All the pattern functions in this file remain
 *          unchanged — only the pin-level primitives differ.
 *
 *          BLOCKING DESIGN
 *          ────────────────
 *          All buzzer functions use HAL_Delay() — they block the calling
 *          context for the duration of the pattern.  This is intentional:
 *            - Buzzer events are infrequent (boot, goal, error, low batt).
 *            - Blocking keeps the implementation simple and reliable.
 *            - The 1 kHz control loop runs from a timer ISR (TIM5), so
 *              it continues uninterrupted even while the main loop blocks.
 *            - Never call buzzer functions from inside the 1 kHz ISR.
 *
 *          PATTERN CATALOGUE
 *          ──────────────────
 *          boot         : 2 short beeps  — "I'm alive"
 *          run_start    : 1 short beep   — "starting a run"
 *          goal         : 1 long beep    — "reached the goal"
 *          error        : 3 long beeps   — "fatal init failure"
 *          low_battery  : 3 rapid beeps  — "swap the pack"
 *          search_done  : 2 medium beeps — "search run complete"
 *          calibrated   : 3 ascending    — "calibration saved"
 *
 *          DEPENDENCIES
 *          ─────────────
 *          pins.h   — BUZZER_ON(), BUZZER_OFF()
 *          config.h — BUZZ_START_MS, BUZZ_BOOT_MS, BUZZ_BOOT_GAP_MS,
 *                     BUZZ_GOAL_MS, BUZZ_ERROR_MS, BUZZ_ERROR_GAP_MS,
 *                     BUZZ_LOWBATT_MS, BUZZ_LOWBATT_GAP_MS
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "error.h"

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the buzzer module.
 *
 * @details Ensures the buzzer pin starts in the OFF state.
 *          The GPIO direction is configured by gpio_init() in main.c
 *          before this is called — this function only guarantees the
 *          pin is LOW on entry.
 *
 * @return MM_OK always.
 */
MmResult_t buzzer_init(void);

/* =========================================================================
 * PRIMITIVE
 * ======================================================================= */

/**
 * @brief  Blocking single beep of specified duration.
 *
 * @details Turns buzzer ON, waits ms milliseconds, turns buzzer OFF.
 *          No gap after the beep — caller controls inter-beep spacing.
 *
 * @warning Blocks the calling thread for ms milliseconds.
 *          Never call from the 1 kHz ISR.
 *
 * @param  ms  Beep duration in milliseconds. Zero = no action.
 */
void buzzer_beep(uint32_t ms);

/**
 * @brief  Blocking repeated beep pattern.
 *
 * @details Produces 'count' beeps, each 'on_ms' ms long, separated
 *          by 'off_ms' ms gaps.  No trailing gap after the last beep.
 *          Total blocking time = count × on_ms + (count − 1) × off_ms.
 *
 * @warning Blocks for the full pattern duration.
 *          Never call from the 1 kHz ISR.
 *
 * @param  count   Number of beeps (0 = no action).
 * @param  on_ms   Duration of each ON phase (ms).
 * @param  off_ms  Duration of each OFF gap between beeps (ms).
 */
void buzzer_pattern(uint8_t count, uint32_t on_ms, uint32_t off_ms);

/* =========================================================================
 * NAMED PATTERNS
 * Durations defined in config.h BUZZ_xxx constants.
 * ======================================================================= */

/**
 * @brief  Boot confirmation — 2 short beeps.
 *
 * @details Pattern: ON(100ms) OFF(80ms) ON(100ms)
 *          Total  : ~280 ms
 *          Meaning: "Firmware started, HAL initialised."
 *          Called once from modules_init() after all modules pass.
 */
void buzzer_boot(void);

/**
 * @brief  Run start acknowledgement — 1 short beep.
 *
 * @details Pattern: ON(50ms)
 *          Total  : 50 ms
 *          Meaning: "Button pressed, run beginning in 1–2 seconds."
 *          Called from modes.c immediately after the button is released
 *          and before the HAL_Delay countdown before motion starts.
 */
void buzzer_run_start(void);

/**
 * @brief  Goal reached — 1 long beep.
 *
 * @details Pattern: ON(500ms)
 *          Total  : 500 ms
 *          Meaning: "Robot has entered the 4-cell goal area."
 *          Called from explorer.c when maze_is_goal() returns true.
 *          The robot pauses briefly at the goal — 500 ms beep fits
 *          within the natural stop time.
 */
void buzzer_goal(void);

/**
 * @brief  Fatal error — 3 long beeps.
 *
 * @details Pattern: ON(300ms) OFF(150ms) × 3
 *          Total  : ~1.05 s
 *          Meaning: "Module initialisation failed — check UART for error code."
 *          Called from fatal_error_handler() in main.c.
 *          After this pattern the LED blinks indefinitely — robot is halted.
 */
void buzzer_error(void);

/**
 * @brief  Low battery warning — 3 rapid triple beeps.
 *
 * @details Pattern: [ON(60ms) OFF(60ms)] × 3
 *          Total  : ~360 ms
 *          Meaning: "Voltage below BATT_LOW_V — swap the pack soon."
 *          Called from scheduler.c when battery_is_low() transitions true.
 *          Only called once per voltage drop — not repeated every 500 ms.
 */
void buzzer_low_battery(void);

/**
 * @brief  Search run complete — 2 medium beeps.
 *
 * @details Pattern: ON(150ms) OFF(100ms) ON(150ms)
 *          Total  : ~400 ms
 *          Meaning: "Flood-fill exploration complete, wall map built."
 *          Called from explorer.c when the robot returns to start after
 *          a successful search run.
 */
void buzzer_search_done(void);

/**
 * @brief  Calibration saved — 3 ascending-length beeps.
 *
 * @details Pattern: ON(80ms) OFF(60ms) ON(120ms) OFF(60ms) ON(200ms)
 *          Total  : ~520 ms
 *          Meaning: "IR calibration data written to Flash."
 *          Ascending duration gives an audible "rising" feel that
 *          distinguishes it from the error pattern (all equal length).
 *          Called from calibration.c after ir_cal_save() returns MM_OK.
 */
void buzzer_calibrated(void);

/* =========================================================================
 * UTILITY
 * ======================================================================= */

/**
 * @brief  Immediately silence the buzzer.
 *
 * @details Forces the buzzer pin LOW regardless of current state.
 *          Use in fatal_error_handler() to ensure the buzzer is off
 *          before starting the LED blink loop, in case a pattern was
 *          interrupted mid-beep by an exception.
 */
void buzzer_off(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
