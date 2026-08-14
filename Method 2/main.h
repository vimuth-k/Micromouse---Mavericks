/**
 * @file    main.h
 * @brief   MicroMaze 3 · Peripheral handle externs and top-level prototypes.
 * @details
 *   WHAT THIS FILE IS
 *   ─────────────────────────────────────────────────────────────────────
 *   The CubeMX-style companion to main.c: every HAL peripheral handle
 *   main.c owns (htim1, htim2, htim3, htim5, hadc1, hdma_adc1, hi2c1,
 *   huart1 — defined in main.c, see the "HAL PERIPHERAL HANDLES" block
 *   there) is declared here as `extern` so any driver can take a
 *   pointer to it without redeclaring the type itself:
 *
 *       imu_init(&hi2c1);
 *       logger_init(&huart1);
 *       oled_init(&hi2c1);
 *
 *   This is why nearly every .c file in the project — motors.c,
 *   battery.c, oled.c, logger.c, imu.c, utils.c, safety.c,
 *   calibration.c, startup_test.c, maze.c, and more — includes
 *   "main.h": it is the one place that both pulls in the HAL headers
 *   (via stm32f4xx_hal.h) and exposes the concrete handle instances by
 *   name. Unlike the rest of this project, this file is standard
 *   CubeMX boilerplate rather than hand-designed application logic —
 *   that is also why it does not appear on the module-by-module file
 *   tree plan: CubeMX would normally generate it automatically the
 *   moment the peripherals are configured in the .ioc file.
 *
 *   fatal_error_handler() (implemented in main.c) is also prototyped
 *   here so any module that hits an unrecoverable condition during its
 *   own init can hand off to the same halt-and-blink routine main.c
 *   uses for its own init failures, rather than each module inventing
 *   its own fatal-error path.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * HAL peripheral handles — defined in main.c, used throughout the project
 * ═══════════════════════════════════════════════════════════════════════ */

extern TIM_HandleTypeDef  htim1;      /**< Motor PWM    — TIM1 CH1 PA8, CH2 PA9 */
extern TIM_HandleTypeDef  htim2;      /**< Left  encoder — TIM2 PA0/PA1 (32-bit) */
extern TIM_HandleTypeDef  htim3;      /**< Right encoder — TIM3 PB4/PB5 (16-bit) */
extern TIM_HandleTypeDef  htim5;      /**< Control loop 1 kHz — TIM5             */
extern ADC_HandleTypeDef  hadc1;      /**< IR + battery ADC scan — ADC1          */
extern DMA_HandleTypeDef  hdma_adc1;  /**< DMA for ADC1 circular scan            */
extern I2C_HandleTypeDef  hi2c1;      /**< MPU6500 + OLED — I2C1 PB8/PB9         */
extern UART_HandleTypeDef huart1;     /**< Debug UART1                           */

/* ═══════════════════════════════════════════════════════════════════════
 * Top-level prototypes
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Halt-and-report handler for an unrecoverable module init
 *         failure. Implemented in main.c.
 *
 * @details Disables motors, logs the failure, shows it on the OLED,
 *          then halts forever. No onboard LED or buzzer is available
 *          on this board revision (PC13 is committed to MPU6500_INT) —
 *          the OLED and UART log are the only error-reporting channels.
 *          Never returns.
 *
 * @param  err  The MmResult_t returned by whichever init call failed.
 */
void fatal_error_handler(MmResult_t err);

/**
 * @brief  Standard CubeMX HAL error hook.
 * @details Not currently called anywhere in this project (no
 *          stm32f4xx_it.c interrupt-handler file exists yet), but
 *          declared here since CubeMX always generates this prototype
 *          and future HAL_ErrorCallback()-style code will expect it to
 *          exist. Implemented in main.c as a thin wrapper around
 *          fatal_error_handler(MM_ERR_DRIVER).
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
