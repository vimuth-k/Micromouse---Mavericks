/**
 * @file    imu.h
 * @brief   MicroMaze 3 · MPU6500 gyroscope driver — yaw-rate integration.
 * @details
 *   WHAT THIS MODULE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   Talks to the MPU6500 over I2C1 (shared bus with the OLED, 400 kHz,
 *   PB8/PB9) and maintains a running heading estimate by integrating the
 *   Z-axis gyro rate once per 1 kHz control-loop tick:
 *
 *       yaw_deg += (gyro_z_dps − zero_rate_offset) × GYRO_DT
 *
 *   Only the Z axis (yaw) is used — this is a ground-plane maze robot,
 *   pitch/roll are not tracked. Motion control (motion.c) reads
 *   imu_yaw_deg() as the heading-PID feedback signal for both straight
 *   driving (hold 0°) and turns (hold target angle).
 *
 *   ZERO-RATE OFFSET
 *   ─────────────────────────────────────────────────────────────────────
 *   Every MPU6500 has a small DC bias on its gyro output even when
 *   perfectly still (typically a few deg/s). Left uncorrected, this
 *   bias integrates into steady heading drift over a run. This module
 *   applies a correctable offset (default 0.0, i.e. uncorrected) that
 *   Calibration/gyro_cal.c sets via imu_set_gyro_offset() after
 *   averaging GYRO_CAL_SAMPLES readings with the robot held still
 *   (see gyro_cal_run() / gyro_cal_get_offset() in main.c). imu_gyro_dps()
 *   exposes the raw (pre-offset) rate specifically so gyro_cal.c has
 *   something to average.
 *
 *   ISR SAFETY
 *   ─────────────────────────────────────────────────────────────────────
 *   imu_update() is called every tick from motion_1khz_tick(), which
 *   itself runs inside the TIM5 1 kHz ISR (see motion.c) — budgeted at
 *   ~20 µs for the 2-byte blocking I2C read. It must
 *   never block longer than the I2C timeout. All other imu_* functions
 *   are safe to call only from main-loop context (init, calibration,
 *   and status queries).
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the MPU6500: wake it from sleep, configure sample
 *         rate / DLPF / full-scale range, and verify WHO_AM_I.
 *
 * @details Configuration applied (values from config.h Section 10):
 *          - PWR_MGMT_1   = 0x01 (wake, PLL with X-gyro reference clock)
 *          - SMPLRT_DIV   = GYRO_SMPLRT_DIV (1 kHz internal sample rate)
 *          - CONFIG       = GYRO_DLPF_REG_VAL (~92 Hz DLPF bandwidth)
 *          - GYRO_CONFIG  = GYRO_FS_REG_VAL (±500 dps full scale)
 *          Resets the internal yaw estimate and gyro offset to 0.
 *
 * @param  hi2c  Pointer to an initialised I2C_HandleTypeDef (hi2c1 from
 *               main.c — the same bus the OLED uses). Stored internally
 *               — must remain valid for the module's lifetime.
 *
 * @return MM_OK          Configuration written and WHO_AM_I matched.
 * @return MM_ERR_PARAM   @p hi2c was NULL.
 * @return MM_ERR_DRIVER  I2C write/read failed, or WHO_AM_I did not
 *                        match MPU6500_WHO_AM_I_VAL — check PB8/PB9
 *                        wiring and the 4.7 kΩ pull-ups shared with
 *                        the OLED.
 */
MmResult_t imu_init(void *hi2c);

/**
 * @brief  Read the current gyro Z rate and integrate it into the yaw
 *         estimate. Call once per 1 kHz control-loop tick.
 *
 * @details Reads GYRO_ZOUT_H/L (2 bytes, burst read), converts to
 *          deg/s using GYRO_SENSITIVITY, stores the raw rate for
 *          imu_gyro_dps(), then advances the yaw estimate by
 *          (raw_dps − offset) × GYRO_DT. On an I2C read failure, the
 *          yaw estimate and cached rate are left unchanged for that
 *          tick rather than integrating a garbage value — a single
 *          dropped sample at 1 kHz is a negligible heading error, and
 *          this function must not block or log from ISR context to
 *          report the fault (see the ISR-safety note in the file
 *          banner).
 *
 * @warning ISR context (TIM5, via motion_1khz_tick()). No logging, no
 *          allocation, ~20 µs budget.
 */
void imu_update(void);

/**
 * @brief  Current integrated heading estimate, in degrees.
 * @details Accumulates without wraparound — motion.c calls
 *          imu_reset_yaw() before each straight move or turn, so the
 *          value only needs to stay valid over one segment's worth of
 *          travel, never across a full run.
 * @return Current yaw estimate (degrees). Positive direction matches
 *         whichever sign convention the mounted gyro Z axis produces
 *         for a clockwise-from-above rotation — verified empirically
 *         during Mode 4 (turn test).
 */
float imu_yaw_deg(void);

/**
 * @brief  Zero the yaw estimate. Call immediately before starting a
 *         straight move (heading-hold reference) or a turn (so the
 *         turn's PID error is measured from 0).
 */
void imu_reset_yaw(void);

/**
 * @brief  Return the WHO_AM_I value read during imu_init().
 * @details Cached at init time so main.c can log/compare it without a
 *          second I2C transaction. Returns 0x00 if imu_init() has not
 *          been called or failed before reaching the WHO_AM_I read.
 * @return The raw WHO_AM_I register value (expect MPU6500_WHO_AM_I_VAL).
 */
uint8_t imu_who_am_i(void);

/**
 * @brief  Most recent raw (pre-offset) gyro Z rate, in deg/s.
 * @details Updated every imu_update() call. Intended for
 *          Calibration/gyro_cal.c to sample and average over
 *          GYRO_CAL_SAMPLES ticks while the robot is held still —
 *          that average becomes the argument to imu_set_gyro_offset().
 * @return Latest raw gyro Z rate (deg/s), before zero-rate correction.
 */
float imu_gyro_dps(void);

/**
 * @brief  Set the zero-rate offset subtracted from every raw gyro
 *         reading before yaw integration.
 * @details Called by Calibration/gyro_cal.c after gyro_cal_run()
 *          finishes averaging. Does not itself touch the current yaw
 *          estimate — call imu_reset_yaw() separately if a clean
 *          heading reference is also needed.
 * @param  offset_dps  Zero-rate bias to subtract, in deg/s.
 */
void imu_set_gyro_offset(float offset_dps);

/**
 * @brief  Return the zero-rate offset currently applied.
 * @details Backs gyro_cal_get_offset() in main.c's boot log.
 * @return Current offset (deg/s), 0.0 if imu_set_gyro_offset() has
 *         never been called (i.e. uncalibrated).
 */
float imu_get_gyro_offset(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
