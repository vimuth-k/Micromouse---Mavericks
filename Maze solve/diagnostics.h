/**
 * @file    diagnostics.h
 * @brief   MicroMaze 3 · Live diagnostic view modes (DIP Modes 0, 11, 12, 13).
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H
#include <stdint.h>
#include "error.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief  Run a diagnostic view for the given DIP mode.
 *
 * @details Handles MODE_MONITOR (0), MODE_GYRO_DEBUG (11), MODE_PRINT_MAZE (12),
 *          and MODE_BATTERY_CHECK (13). Live views block until the button is pressed;
 *          MODE_PRINT_MAZE is a one-shot dump.
 *
 * @param  mode  Raw DIP switch mode value.
 * @return MM_OK always.
 */
MmResult_t diagnostics_run(uint8_t mode);
#ifdef __cplusplus
}
#endif
#endif /* DIAGNOSTICS_H */
