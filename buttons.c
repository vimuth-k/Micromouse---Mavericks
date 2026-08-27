/**
 * @file    buttons.c
 * @brief   MicroMaze 3 · Debounced driver for the onboard test/user button.
 *
 * @author  VDawn
 * @date    2026
 */

#include "buttons.h"
#include "pins.h"
#include "config.h"
#include "main.h"

static bool s_stable_pressed = false;
static bool s_last_raw = false;
static uint32_t s_change_started_ms = 0U;

static bool s_just_pressed  = false;
static bool s_just_released = false;

void buttons_init(void)
{
    bool raw = BTN_PRESSED();

    s_stable_pressed    = raw;
    s_last_raw          = raw;
    s_change_started_ms = HAL_GetTick();
    s_just_pressed      = false;
    s_just_released     = false;
}

void buttons_update(void)
{
    bool raw = BTN_PRESSED();

    if (raw != s_last_raw)
    {
        s_last_raw          = raw;
        s_change_started_ms = HAL_GetTick();
    }
    else if (raw != s_stable_pressed)
    {
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
