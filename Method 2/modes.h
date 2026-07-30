/**
 * @file    modes.h
 * @brief   MicroMaze 3 · DIP-mode dispatch layer — one function per mode.
 * @details
 *   WHAT THIS MODULE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   main.c reads the DIP switches once at boot and calls exactly one
 *   modes_run_*() function via run_selected_mode()'s switch statement.
 *   Each function here is a thin wrapper around whichever module
 *   already does the real work (calibration.c, wall_follow.c,
 *   explorer.c, speedrun.c, diagnostics.c, startup_test.c) — modes.c's
 *   own job is: call the right sequence of functions, drive
 *   state_machine_transition() around the run-level states
 *   (SEARCHING/RETURNING/SPEEDRUNNING — see state_machine.h), and
 *   never return.
 *
 *   THE "NEVER RETURN" CONTRACT
 *   ─────────────────────────────────────────────────────────────────────
 *   main.c's run_selected_mode() documents every mode function as
 *   either looping forever or finishing its task and then halting in
 *   an idle loop — if a modes_run_*() function ever actually returns,
 *   main.c treats that as an error condition (disables motors, halts).
 *   Every function here ends by calling the shared
 *   modes_idle_forever() helper instead of returning: it shows a final
 *   result on the OLED, logs it, and then loops (servicing the
 *   scheduler and blinking the LED) until the operator resets the
 *   board to select a different DIP mode. DIP switches are only read
 *   once at boot — there is no live mode-switching without a reset.
 *
 *   TWO MODES WITH NO DEDICATED MODULE
 *   ─────────────────────────────────────────────────────────────────────
 *   MODE_MOTOR_TEST and MODE_STRAIGHT_TEST are simple enough that they
 *   don't have their own .c file — they're implemented directly in
 *   modes.c using motors.h/encoders.h/motion.h, similarly to how
 *   startup_test.c's motor/encoder check works but applied to one
 *   motor at a time (MOTOR_TEST) or as a single longer straight-line
 *   drift measurement (STRAIGHT_TEST).
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef MODES_H
#define MODES_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the modes module.
 * @details Currently stateless. Kept as its own init call for
 *          lifecycle consistency with every other module.
 * @return MM_OK always.
 */
MmResult_t modes_init(void);

/** DIP Mode 0 — live IR sensor view. Wraps diagnostics_run_monitor(). */
void modes_run_monitor(void);

/** DIP Mode 1 — interactive IR calibration. Wraps calibration_run_ir(). */
void modes_run_ir_calibrate(void);

/** DIP Mode 2 — spin each motor in turn, verify its encoder counts. */
void modes_run_motor_test(void);

/** DIP Mode 3 — drive STRAIGHT_TEST_CELLS cells, report yaw drift. */
void modes_run_straight_test(void);

/** DIP Mode 4 — 90/180/360° turn accuracy check. Wraps turn_run_test_sequence(). */
void modes_run_turn_test(void);

/** DIP Mode 5 — left-hand-rule fallback solver. Wraps wall_follow_run(). */
void modes_run_wall_follower(void);

/**
 * @brief  DIP Mode 6 — flood-fill search to the goal, then return to
 *         start.
 * @details Drives state_machine_transition() through
 *          SEARCHING → RETURNING → IDLE (or → ERROR on failure) around
 *          explorer_search_to_goal() / explorer_return_to_start().
 */
void modes_run_search(void);

/**
 * @brief  DIP Modes 7/8/9 — execute the known optimised path at a
 *         fixed speed.
 * @details One handler for all three speed-run modes; main.c passes
 *          SPD_RUN1/2/3 as @p speed_mmps. Drives
 *          state_machine_transition() through
 *          SPEEDRUNNING → IDLE (or → ERROR) around speedrun_run().
 * @param  speed_mmps  Target cruise speed for this run.
 */
void modes_run_speed(float speed_mmps);

/**
 * @brief  DIP Mode 10 — full qualifier sequence: search, return, then
 *         three speed runs back to back (SPD_RUN1, SPD_RUN2, SPD_RUN3).
 * @details Aborts the remaining sequence if the search phase fails —
 *          there is no known path to attempt a speed run against.
 *          Drives state_machine_transition() through every state in
 *          turn: SEARCHING → RETURNING → IDLE → SPEEDRUNNING → IDLE →
 *          SPEEDRUNNING → IDLE → SPEEDRUNNING → FINISHED.
 */
void modes_run_auto_qualifier(void);

/** DIP Mode 11 — live yaw/rate/offset view. Wraps diagnostics_run_gyro_debug(). */
void modes_run_gyro_debug(void);

/** DIP Mode 12 — one-shot wall-map dump via UART. Wraps diagnostics_print_maze(). */
void modes_run_print_maze(void);

/** DIP Mode 13 — live battery voltage/percentage view. Wraps diagnostics_run_battery_check(). */
void modes_run_battery_check(void);

/** DIP Mode 14 — full hardware self-test. Wraps startup_test_run(). */
void modes_run_startup_test(void);

#ifdef __cplusplus
}
#endif

#endif /* MODES_H */
