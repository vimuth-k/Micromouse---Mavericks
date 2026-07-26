/**
 * @file    calibration.c
 * @brief   MicroMaze 3 · IR + gyro calibration routines — implementation.
 * @details See calibration.h for the full design rationale behind both
 *          routines, including why gyro_cal_run() drives its own
 *          sample loop instead of relying on the (not-yet-running)
 *          TIM5 ISR.
 *
 * @author  VDawn
 * @date    2026
 */
#include "calibration.h"
#include "config.h"
#include "error.h"
#include "logger.h"
#include "ir.h"
#include "imu.h"
#include "oled.h"
#include "main.h"     /* HAL_Delay */

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t calibration_run_ir(void)
{
    LOG_INFO("IR calibration: step 1/2 — place robot in open space");
    oled_show_message("IR CAL 1/2", "PLACE: OPEN");
    HAL_Delay(CAL_IR_STEP_DELAY_MS + CAL_IR_SETTLE_MS);
    ir_cal_ambient();

    LOG_INFO("IR calibration: step 2/2 — place robot in start cell");
    oled_show_message("IR CAL 2/2", "PLACE: START");
    HAL_Delay(CAL_IR_STEP_DELAY_MS + CAL_IR_SETTLE_MS);
    ir_cal_wall();

    MmResult_t result = ir_cal_save();

    if (result == MM_OK)
    {
        LOG_INFO("IR calibration saved to Flash");
        oled_show_message("IR CAL", "SAVED OK");
    }
    else
    {
        LOG_ERROR("IR calibration Flash save failed: %d", (int)result);
        oled_show_error("CAL FAILED", (int32_t)result);
    }

    return result;
}

MmResult_t gyro_cal_run(void)
{
    float sum_dps = 0.0f;

    for (uint32_t i = 0U; i < GYRO_CAL_SAMPLES; i++)
    {
        /* Drive the IMU ourselves — the TIM5 ISR that normally calls
         * imu_update() every ms is not running yet at this point in
         * boot (gyro_cal_run() executes before motion_init()). */
        imu_update();
        sum_dps += imu_gyro_dps();
        HAL_Delay(1U);
    }

    float offset_dps = sum_dps / (float)GYRO_CAL_SAMPLES;
    imu_set_gyro_offset(offset_dps);

    return MM_OK;
}

float gyro_cal_get_offset(void)
{
    return imu_get_gyro_offset();
}
