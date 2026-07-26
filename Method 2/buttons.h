/**
 * @file    buttons.h
 * @brief   MicroMaze 3 · Debounced driver for the onboard test/user button.
 * @details
 *   HARDWARE
 *   ─────────────────────────────────────────────────────────────────────
 *   One physical push-button (PC15, active-low, internal pull-up) —
 *   separate from the 4-position DIP switch used for mode selection.
 *   pins.h already provides the raw primitives this module builds on:
 *
 *       BTN_PRESSED()     Instantaneous, undebounced pin state.
 *       BTN_WAIT_PRESS()  Crude blocking wait — no debounce, no timeout.
 *
 *   WHAT THIS MODULE ADDS
 *   ─────────────────────────────────────────────────────────────────────
 *   Mechanical switches bounce for a few milliseconds on every press
 *   and release, which BTN_PRESSED() reports as-is — reading it directly
 *   in a tight loop can see several spurious transitions per real press.
 *   buttons_update() runs a simple debounce (BTN_DEBOUNCE_MS from
 *   config.h) and exposes the result as a stable state plus one-shot
 *   press/release edges, and buttons_wait_for_press() wraps that into a
 *   simple blocking "wait for a real confirm press, with a timeout" call
 *   — the piece calibration.c's IR calibration sequence is currently
 *   missing (it uses a fixed delay instead, see calibration.h).
 *
 *   TWO WAYS TO USE IT
 *   ─────────────────────────────────────────────────────────────────────
 *   1. Non-blocking: call buttons_update() periodically (e.g. from a
 *      scheduler slot), then poll buttons_just_pressed() /
 *      buttons_is_pressed() elsewhere without stalling the caller.
 *   2. Blocking: call buttons_wait_for_press(timeout_ms) directly — it
 *      drives its own buttons_update() polling loop internally via
 *      HAL_Delay(), so the caller doesn't need a scheduler slot set up
 *      first. This is what calibration.c would use.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the debounce state machine.
 * @details Reads the current raw pin state as the initial stable state
 *          (so a button already held at boot doesn't register a false
 *          "just pressed" edge on the first buttons_update() call).
 *          GPIO configuration itself is done in main.c; this only
 *          resets this module's internal debounce state.
 */
void buttons_init(void);

/**
 * @brief  Run one debounce step. Call periodically — either from a
 *         scheduler slot (non-blocking use) or in a loop from
 *         buttons_wait_for_press() (blocking use).
 * @details Samples BTN_PRESSED() and, if the raw state has differed
 *          from the current stable state for at least BTN_DEBOUNCE_MS,
 *          accepts the change and sets the corresponding one-shot edge
 *          flag. Cheap — safe to call at any reasonable rate (a few ms
 *          up to tens of ms between calls).
 */
void buttons_update(void);

/**
 * @brief  Current debounced state.
 * @return true if the button is currently held down (debounced).
 */
bool buttons_is_pressed(void);

/**
 * @brief  Did the button transition to pressed since the last call to
 *         this function?
 * @details One-shot: reading this clears the flag, so each real press
 *          is reported exactly once regardless of how often
 *          buttons_update() runs in between.
 * @return true exactly once per debounced press-down transition.
 */
bool buttons_just_pressed(void);

/**
 * @brief  Did the button transition to released since the last call to
 *         this function?
 * @details One-shot, same semantics as buttons_just_pressed().
 * @return true exactly once per debounced release transition.
 */
bool buttons_just_released(void);

/**
 * @brief  Block until a debounced press occurs or @p timeout_ms elapses.
 *
 * @details Drives its own buttons_update() polling loop (roughly every
 *          BTN_DEBOUNCE_MS) via HAL_Delay() — no scheduler slot needed.
 *          Consumes the press edge it detects, so a subsequent
 *          buttons_just_pressed() call will not also report it.
 *
 * @warning Blocking. Main-loop context only, never from an ISR.
 *
 * @param  timeout_ms  Maximum time to wait, in ms. 0 means poll once
 *                      and return immediately without waiting.
 *
 * @return true   A debounced press was detected within the timeout.
 * @return false  @p timeout_ms elapsed with no press detected.
 */
bool buttons_wait_for_press(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUTTONS_H */
