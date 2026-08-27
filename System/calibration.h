/**
 * @file    calibration.h
 * @brief   MicroMaze 3 · IR + gyro calibration routines.
 * @details
 *   TWO INDEPENDENT ROUTINES
 *   ─────────────────────────────────────────────────────────────────────
 *   1. calibration_run_ir() — interactive, operator-triggered by
 *      selecting DIP Mode 1 (MODE_IR_CALIBRATE). Two-step sequence:
 *        Step 1: robot in open space  → ir_cal_ambient()
 *        Step 2: robot in start cell  → ir_cal_wall()
 *      followed by ir_cal_save() to persist to Flash. Each step shows
 *      an OLED prompt, then waits on buttons_wait_for_press() (up to
 *      CAL_IR_CONFIRM_TIMEOUT_MS) so the operator can take as long as
 *      they need to position the robot before confirming — if the
 *      button is not pressed within the timeout, calibration.c logs a
 *      warning and proceeds anyway rather than hanging indefinitely.
 *
 *   2. gyro_cal_run() — automatic, called unconditionally from main.c's
 *      boot sequence every single boot, before motion_init() starts the
 *      TIM5 control loop. Averages GYRO_CAL_SAMPLES raw gyro-Z readings
 *      (config.h Section 10, currently 500 samples ≈ 500 ms) while the
 *      robot is held still, then calls imu_set_gyro_offset() with the
 *      result. Because the TIM5 ISR is not running yet at this point in
 *      boot, gyro_cal_run() drives its own 1 ms sample loop by calling
 *      imu_update() directly rather than relying on the ISR to do it.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Run the interactive two-step IR calibration sequence and
 *         persist the result to Flash.
 *
 * @details Sequence:
 *          1. OLED: "IR CAL 1/2" / "PRESS: OPEN", wait for a confirm
 *             press (buttons_wait_for_press(), up to
 *             CAL_IR_CONFIRM_TIMEOUT_MS), then CAL_IR_SETTLE_MS, then
 *             call ir_cal_ambient().
 *          2. OLED: "IR CAL 2/2" / "PRESS: START", same wait/settle,
 *             then call ir_cal_wall().
 *          3. ir_cal_save() to Flash sector 7 (coordinated with the
 *             maze region via flash_storage.c — see flash_storage.h).
 *          4. OLED shows the result ("CAL OK" or "CAL FAILED").
 *
 *          Call from modes_run_ir_calibrate() when the operator selects
 *          DIP Mode 1. Blocks until both confirm presses arrive (or
 *          time out) plus roughly 2 × CAL_IR_SETTLE_MS and the Flash
 *          write (~100 ms) — main-loop context only, never call from
 *          an ISR.
 *
 * @return MM_OK           Both steps captured and saved successfully.
 * @return MM_ERR_STORAGE  ir_cal_save() failed to write Flash.
 */
MmResult_t calibration_run_ir(void);

/**
 * @brief  Measure the gyro's zero-rate bias and apply it to imu.c.
 *
 * @details Averages GYRO_CAL_SAMPLES consecutive imu_gyro_dps() readings,
 *          driving one imu_update() per sample itself (~1 ms apart via
 *          HAL_Delay(1), matching GYRO_DT) since the TIM5 ISR that
 *          normally drives imu_update() is not yet running at this
 *          point in boot. Calls imu_set_gyro_offset() with the result.
 *          The robot must be held perfectly still for the ~500 ms this
 *          takes — main.c already prompts for this via the OLED before
 *          calling this function.
 *
 * @return MM_OK  Always — there is no failure mode for averaging gyro
 *                samples that are already being read successfully by
 *                imu_update() (an I2C fault there just contributes a
 *                stale/zero sample for that tick rather than aborting).
 */
MmResult_t gyro_cal_run(void);

/**
 * @brief  Return the gyro zero-rate offset most recently computed by
 *         gyro_cal_run().
 * @details Thin passthrough to imu_get_gyro_offset() — kept as its own
 *          entry point because main.c's boot log already calls it by
 *          this name.
 * @return Offset in deg/s, 0.0 if gyro_cal_run() has not been called.
 */
float gyro_cal_get_offset(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */
