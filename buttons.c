/**
 * @file    buttons.c
 * @brief   MicroMaze 3 · Debounced driver for the onboard test/user button.
 * @details See buttons.h for the module overview and the two usage
 *          patterns (polled vs. blocking-with-timeout).
 *
 * @author  VDawn
 * @date    2026
 */
#include "buttons.h"
#include "pins.h"     /* BTN_PRESSED() */
#include "config.h"   /* BTN_DEBOUNCE_MS */
#include "main.h"     /* HAL_GetTick(), HAL_Delay() */

/* ═══════════════════════════════════════════════════════════════════════
 * Module state
 * ═══════════════════════════════════════════════════════════════════════ */

/** Debounced (accepted) button state. */
static bool s_stable_pressed = false;

/** Most recent raw reading, tracked to detect the start of a change. */
static bool s_last_raw = false;

/** HAL_GetTick() when s_last_raw most recently changed value. */
static uint32_t s_change_started_ms = 0U;

/** One-shot edge flags, cleared on read. */
static bool s_just_pressed  = false;
static bool s_just_released = false;

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

void buttons_init(void)
{
    bool raw = BTN_PRESSED();

    s_stable_pressed    = raw;
    s_last_raw           = raw;
    s_change_started_ms  = HAL_GetTick();
    s_just_pressed       = false;
    s_just_released      = false;
}

void buttons_update(void)
{
    bool raw = BTN_PRESSED();

    if (raw != s_last_raw)
    {
        /* Raw state just changed — start (or restart) the debounce
         * timer for this new candidate state. */
        s_last_raw          = raw;
        s_change_started_ms = HAL_GetTick();
    }
    else if (raw != s_stable_pressed)
    {
        /* Raw state has been steady and differs from the last accepted
         * stable state — accept it once it's held for BTN_DEBOUNCE_MS. */
        if ((HAL_GetTick() - s_change_started_ms) >= BTN_DEBOUNCE_MS)
        {
            s_stable_pressed = raw;

            if (raw)
            {
                s_just_pressed = true;
            }
            else
            {
                s_just_released = true;
            }
        }
    }
    /* else: raw == s_last_raw == s_stable_pressed — steady state, nothing to do. */
}

bool buttons_is_pressed(void)
{
    return s_stable_pressed;
}

bool buttons_just_pressed(void)
{
    bool edge      = s_just_pressed;
    s_just_pressed = false;
    return edge;
}

bool buttons_just_released(void)
{
    bool edge        = s_just_released;
    s_just_released  = false;
    return edge;
}

bool buttons_wait_for_press(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (true)
    {
        buttons_update();

        if (buttons_just_pressed())
        {
            return true;
        }

        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return false;
        }

        HAL_Delay(BTN_DEBOUNCE_MS);
    }
}
