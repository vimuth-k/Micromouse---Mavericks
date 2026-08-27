/**
 * @file    modes.h
 * @brief   MicroMaze 3 · DIP-mode dispatcher and execution coordinator.
 * @details
 *   WHAT THIS MODULE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   main.c reads the DIP switches once at boot and invokes modes_run(mode).
 *   This module acts as the top-level mode dispatcher and coordinator:
 *   it dispatches directly to the selected subsystem (diagnostics,
 *   calibration, explorer, speedrun, wall_follow, startup_test),
 *   coordinates multi-step run flows, drives state_machine_transition()
 *   around run-level states (SEARCHING/RETURNING/SPEEDRUNNING), and halts
 *   in an idle loop when finished.
 *
 *   THE "NEVER RETURN" CONTRACT
 *   ─────────────────────────────────────────────────────────────────────
 *   modes_run() does not return under normal operation. Every mode
 *   either loops indefinitely or completes its task and enters the
 *   shared idle loop (servicing the scheduler and background tasks)
 *   until the operator resets the microcontroller.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef MODES_H
#define MODES_H

#include <stdint.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the modes module.
 * @return MM_OK always.
 */
MmResult_t modes_init(void);

/**
 * @brief  Dispatch and execute the operating mode selected by DIP switches.
 * @details Dispatches directly to the appropriate subsystem or mode handler,
 *          drives state machine transitions, and enters an idle loop when
 *          complete. Never returns under normal operation.
 * @param  mode  DIP switch value 0–15 (from READ_DIP_SWITCHES()).
 */
void modes_run(uint8_t mode);

/**
 * @brief  Alias for modes_run(mode).
 * @param  mode  DIP switch value 0–15.
 */
void modes_dispatch(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* MODES_H */
