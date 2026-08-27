/**
 * @file    imu.c
 * @brief   MicroMaze 3 · MPU6500 gyroscope driver — yaw-rate integration.
 *
 * @author  VDawn
 * @date    2026
 */

#include "imu.h"
#include "config.h"   /* MPU6500_I2C_ADDR, GYRO_*, ... */
#include "error.h"
#include "main.h"     /* I2C_HandleTypeDef, HAL_I2C_Mem_Read/Write */

#define MPU6500_REG_SMPLRT_DIV     0x19U
#define MPU6500_REG_CONFIG         0x1AU  /**< DLPF_CFG bits [2:0]        */
#define MPU6500_REG_GYRO_CONFIG    0x1BU  /**< FS_SEL bits [4:3]          */
#define MPU6500_REG_GYRO_ZOUT_H    0x47U  /**< ZOUT_L follows at +1       */
#define MPU6500_REG_PWR_MGMT_1     0x6BU
#define MPU6500_REG_WHO_AM_I       0x75U

#define MPU6500_PWR_MGMT_1_WAKE_PLL  0x01U
#define I2C_TIMEOUT_MS              10U

static I2C_HandleTypeDef *s_hi2c          = NULL;
static uint8_t             s_who_am_i     = 0x00U;
static float                s_yaw_deg      = 0.0f;
static float                s_gyro_dps     = 0.0f;
static float                s_gyro_offset  = 0.0f;

static HAL_StatusTypeDef reg_write(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(MPU6500_I2C_ADDR << 1U),
                              reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(s_hi2c, (uint16_t)(MPU6500_I2C_ADDR << 1U),
                             reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, I2C_TIMEOUT_MS);
}

MmResult_t imu_init(void *hi2c)
{
    if (hi2c == NULL)
    {
        return MM_ERR_PARAM;
    }

    s_hi2c        = (I2C_HandleTypeDef *)hi2c;
    s_yaw_deg     = 0.0f;
    s_gyro_dps    = 0.0f;
    s_gyro_offset = 0.0f;

    if (reg_write(MPU6500_REG_PWR_MGMT_1, MPU6500_PWR_MGMT_1_WAKE_PLL) != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }

    HAL_Delay(10U);

    if (reg_write(MPU6500_REG_SMPLRT_DIV, GYRO_SMPLRT_DIV) != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }
    if (reg_write(MPU6500_REG_CONFIG, GYRO_DLPF_REG_VAL) != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }
    if (reg_write(MPU6500_REG_GYRO_CONFIG, GYRO_FS_REG_VAL) != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }

    if (reg_read(MPU6500_REG_WHO_AM_I, &s_who_am_i, 1U) != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }

    if (s_who_am_i != MPU6500_WHO_AM_I_VAL)
    {
        return MM_ERR_DRIVER;
    }

    return MM_OK;
}

void imu_update(void)
{
    uint8_t raw[2];

    if (reg_read(MPU6500_REG_GYRO_ZOUT_H, raw, 2U) != HAL_OK)
    {
        return;
    }

    int16_t raw_z = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    s_gyro_dps    = (float)raw_z / GYRO_SENSITIVITY;

    s_yaw_deg += (s_gyro_dps - s_gyro_offset) * GYRO_DT;
}

float imu_yaw_deg(void)
{
    return s_yaw_deg;
}

void imu_reset_yaw(void)
{
    s_yaw_deg = 0.0f;
}

uint8_t imu_who_am_i(void)
{
    return s_who_am_i;
}

float imu_gyro_dps(void)
{
    return s_gyro_dps;
}

void imu_set_gyro_offset(float offset_dps)
{
    s_gyro_offset = offset_dps;
}

float imu_get_gyro_offset(void)
{
    return s_gyro_offset;
}
