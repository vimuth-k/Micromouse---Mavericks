/**
 * @file    buttons.h
 * @brief   MicroMaze 3 · Debounced driver for the onboard test/user button.
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

void buttons_init(void);
void buttons_update(void);
bool buttons_is_pressed(void);
bool buttons_just_pressed(void);
bool buttons_just_released(void);
bool buttons_wait_for_press(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUTTONS_H */
