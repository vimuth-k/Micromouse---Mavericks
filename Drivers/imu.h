/**
 * @file    imu.h
 * @brief   MicroMaze 3 · MPU6500 gyroscope driver — yaw-rate integration.
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

MmResult_t imu_init(void *hi2c);
void       imu_update(void);
float      imu_yaw_deg(void);
void       imu_reset_yaw(void);
uint8_t    imu_who_am_i(void);
float      imu_gyro_dps(void);
void       imu_set_gyro_offset(float offset_dps);
float      imu_get_gyro_offset(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
