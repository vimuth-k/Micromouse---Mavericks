/**
 * @file    imu.c
 * @brief   MicroMaze 3 · MPU6500 gyroscope driver — yaw-rate integration.
 * @details See imu.h for the module overview, the zero-rate offset
 *          design, and the ISR-safety requirements on imu_update().
 *
 * @author  VDawn
 * @date    2026
 */
#include "imu.h"
#include "config.h"   /* MPU6500_I2C_ADDR, GYRO_*, ... */
#include "error.h"
#include "main.h"     /* I2C_HandleTypeDef, HAL_I2C_Mem_Read/Write */

/* ═══════════════════════════════════════════════════════════════════════
 * MPU6500 register map (subset actually used by this driver)
 * ═══════════════════════════════════════════════════════════════════════ */

#define MPU6500_REG_SMPLRT_DIV     0x19U
#define MPU6500_REG_CONFIG         0x1AU  /**< DLPF_CFG bits [2:0]        */
#define MPU6500_REG_GYRO_CONFIG    0x1BU  /**< FS_SEL bits [4:3]          */
#define MPU6500_REG_GYRO_ZOUT_H    0x47U  /**< ZOUT_L follows at +1       */
#define MPU6500_REG_PWR_MGMT_1     0x6BU
#define MPU6500_REG_WHO_AM_I       0x75U

/** PWR_MGMT_1 = wake from sleep, PLL with X-gyro reference (CLKSEL=1). */
#define MPU6500_PWR_MGMT_1_WAKE_PLL  0x01U

/** Blocking I2C transaction timeout. imu_update() runs in the TIM5 ISR
 *  with a ~20 us budget for this read — 10 ms is a fault ceiling, not
 *  an expected duration; a real timeout here means the bus is stuck. */
#define I2C_TIMEOUT_MS              10U

/* ═══════════════════════════════════════════════════════════════════════
 * Module state
 * ═══════════════════════════════════════════════════════════════════════ */

static I2C_HandleTypeDef *s_hi2c          = NULL;
static uint8_t             s_who_am_i     = 0x00U;
static float                s_yaw_deg      = 0.0f;
static float                s_gyro_dps     = 0.0f; /**< Latest raw (pre-offset) rate. */
static float                s_gyro_offset  = 0.0f; /**< Set by gyro_cal.c.            */

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Write one byte to an MPU6500 register.
 * @return HAL status from HAL_I2C_Mem_Write().
 */
static HAL_StatusTypeDef reg_write(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(s_hi2c, (uint16_t)(MPU6500_I2C_ADDR << 1U),
                              reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, I2C_TIMEOUT_MS);
}

/**
 * @brief  Read @p len consecutive bytes starting at register @p reg.
 * @return HAL status from HAL_I2C_Mem_Read().
 */
static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(s_hi2c, (uint16_t)((MPU6500_I2C_ADDR << 1U) | 1U),
                             reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, I2C_TIMEOUT_MS);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

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

    /* Wake the sensor — it powers up in sleep mode. */
    if (reg_write(MPU6500_REG_PWR_MGMT_1, MPU6500_PWR_MGMT_1_WAKE_PLL)
        != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }

    /* Datasheet-recommended settle time after a clock-source change
     * before further register writes. */
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
        /* Drop this sample rather than integrate a garbage value —
         * see the fault-handling note in imu.h. Never log here: this
         * runs inside the TIM5 ISR. */
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
