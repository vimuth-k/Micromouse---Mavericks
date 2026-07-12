/**
 * @file pins.h
 * @brief Hardware pin and peripheral mapping for the STM32F411CEU6 Black Pill
 * Micromouse controller.
 *
 * This file is the single source of truth for all GPIO pins, alternate
 * functions, peripheral instances, timer channels, and ADC channel
 * assignments. It contains hardware mapping only and intentionally excludes
 * driver code, executable statements, timing configuration, and application
 * logic.
 *
 * @copyright Copyright (c) 2026
 */

#ifndef PINS_H
#define PINS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ========================================================================== */
/* Peripheral instances                                                        */
/* ========================================================================== */

#define PIN_SENSOR_ADC_INSTANCE        ADC1 /**< ADC peripheral for IR and battery sensing. */
#define PIN_IMU_SPI_INSTANCE           SPI2 /**< SPI peripheral for the MPU6500. */
#define PIN_OLED_I2C_INSTANCE          I2C1 /**< I2C peripheral for the SSD1306 OLED. */

/* Motor driver: TB6612FNG                                                     */
/* ========================================================================== */

#define PIN_LEFT_MOTOR_IN1_PORT        GPIOB /**< TB6612FNG AIN1 GPIO port. */
#define PIN_LEFT_MOTOR_IN1_PIN         GPIO_PIN_12 /**< TB6612FNG AIN1 GPIO pin. */
#define PIN_LEFT_MOTOR_IN2_PORT        GPIOB /**< TB6612FNG AIN2 GPIO port. */
#define PIN_LEFT_MOTOR_IN2_PIN         GPIO_PIN_13 /**< TB6612FNG AIN2 GPIO pin. */
#define PIN_RIGHT_MOTOR_IN1_PORT       GPIOB /**< TB6612FNG BIN1 GPIO port. */
#define PIN_RIGHT_MOTOR_IN1_PIN        GPIO_PIN_14 /**< TB6612FNG BIN1 GPIO pin. */
#define PIN_RIGHT_MOTOR_IN2_PORT       GPIOB /**< TB6612FNG BIN2 GPIO port. */
#define PIN_RIGHT_MOTOR_IN2_PIN        GPIO_PIN_15 /**< TB6612FNG BIN2 GPIO pin. */

#define PIN_TB6612_STBY_PORT           GPIOB /**< TB6612FNG STBY GPIO port. */
#define PIN_TB6612_STBY_PIN            GPIO_PIN_11 /**< TB6612FNG STBY GPIO pin. */

/* ========================================================================== */
/* Motor PWM: TIM1                                                            */
/* ========================================================================== */

#define PIN_MOTOR_PWM_TIMER            TIM1 /**< Shared motor PWM timer instance. */
#define PIN_LEFT_MOTOR_PWM_CHANNEL     TIM_CHANNEL_1 /**< TIM1 channel for left motor PWM. */
#define PIN_RIGHT_MOTOR_PWM_CHANNEL    TIM_CHANNEL_2 /**< TIM1 channel for right motor PWM. */

#define PIN_LEFT_MOTOR_PWM_PORT        GPIOA /**< Left motor PWM GPIO port. */
#define PIN_LEFT_MOTOR_PWM_PIN         GPIO_PIN_8 /**< Left motor PWM GPIO pin. */
#define PIN_LEFT_MOTOR_PWM_AF          GPIO_AF1_TIM1 /**< PA8 alternate function: TIM1_CH1. */
#define PIN_RIGHT_MOTOR_PWM_PORT       GPIOA /**< Right motor PWM GPIO port. */
#define PIN_RIGHT_MOTOR_PWM_PIN        GPIO_PIN_9 /**< Right motor PWM GPIO pin. */
#define PIN_RIGHT_MOTOR_PWM_AF         GPIO_AF1_TIM1 /**< PA9 alternate function: TIM1_CH2. */

/* ========================================================================== */
/* Encoders: TIM2 and TIM4                                                    */
/* ========================================================================== */

#define PIN_LEFT_ENCODER_TIMER         TIM2 /**< Left encoder timer instance. */
#define PIN_LEFT_ENCODER_CHANNEL_A     TIM_CHANNEL_1 /**< Left encoder timer channel A. */
#define PIN_LEFT_ENCODER_CHANNEL_B     TIM_CHANNEL_2 /**< Left encoder timer channel B. */
#define PIN_LEFT_ENCODER_A_PORT        GPIOA /**< Left encoder channel A GPIO port. */
#define PIN_LEFT_ENCODER_A_PIN         GPIO_PIN_0 /**< Left encoder channel A GPIO pin. */
#define PIN_LEFT_ENCODER_A_AF          GPIO_AF1_TIM2 /**< PA0 alternate function: TIM2_CH1. */
#define PIN_LEFT_ENCODER_B_PORT        GPIOA /**< Left encoder channel B GPIO port. */
#define PIN_LEFT_ENCODER_B_PIN         GPIO_PIN_1 /**< Left encoder channel B GPIO pin. */
#define PIN_LEFT_ENCODER_B_AF          GPIO_AF1_TIM2 /**< PA1 alternate function: TIM2_CH2. */

#define PIN_RIGHT_ENCODER_TIMER        TIM4 /**< Right encoder timer instance. */
#define PIN_RIGHT_ENCODER_CHANNEL_A    TIM_CHANNEL_1 /**< Right encoder timer channel A. */
#define PIN_RIGHT_ENCODER_CHANNEL_B    TIM_CHANNEL_2 /**< Right encoder timer channel B. */
#define PIN_RIGHT_ENCODER_A_PORT       GPIOB /**< Right encoder channel A GPIO port. */
#define PIN_RIGHT_ENCODER_A_PIN        GPIO_PIN_6 /**< Right encoder channel A GPIO pin. */
#define PIN_RIGHT_ENCODER_A_AF         GPIO_AF2_TIM4 /**< PB6 alternate function: TIM4_CH1. */
#define PIN_RIGHT_ENCODER_B_PORT       GPIOB /**< Right encoder channel B GPIO port. */
#define PIN_RIGHT_ENCODER_B_PIN        GPIO_PIN_7 /**< Right encoder channel B GPIO pin. */
#define PIN_RIGHT_ENCODER_B_AF         GPIO_AF2_TIM4 /**< PB7 alternate function: TIM4_CH2. */

/* ========================================================================== */
/* IR emitters                                                                 */
/* ========================================================================== */

#define PIN_IR_EMIT_DIAG_RIGHT_PORT    GPIOC /**< Right diagonal emitter GPIO port. */
#define PIN_IR_EMIT_DIAG_RIGHT_PIN     GPIO_PIN_1 /**< Right diagonal emitter GPIO pin. */
#define PIN_IR_EMIT_DIAG_LEFT_PORT     GPIOA /**< Left diagonal emitter GPIO port. */
#define PIN_IR_EMIT_DIAG_LEFT_PIN      GPIO_PIN_11 /**< Left diagonal emitter GPIO pin. */
#define PIN_IR_EMIT_FRONT_RIGHT_PORT   GPIOA /**< Front-right emitter GPIO port. */
#define PIN_IR_EMIT_FRONT_RIGHT_PIN    GPIO_PIN_15 /**< Front-right emitter GPIO pin. */
#define PIN_IR_EMIT_FRONT_LEFT_PORT    GPIOB /**< Front-left emitter GPIO port. */
#define PIN_IR_EMIT_FRONT_LEFT_PIN     GPIO_PIN_3 /**< Front-left emitter GPIO pin. */
#define PIN_IR_EMIT_SIDE_RIGHT_PORT    GPIOB /**< Right-side emitter GPIO port. */
#define PIN_IR_EMIT_SIDE_RIGHT_PIN     GPIO_PIN_4 /**< Right-side emitter GPIO pin. */
#define PIN_IR_EMIT_SIDE_LEFT_PORT     GPIOB /**< Left-side emitter GPIO port. */
#define PIN_IR_EMIT_SIDE_LEFT_PIN      GPIO_PIN_5 /**< Left-side emitter GPIO pin. */

/* ========================================================================== */
/* IR receivers: ADC1                                                          */
/* ========================================================================== */

/* ADC input assignments */
#define PIN_IR_FRONT_LEFT_PORT         GPIOA /**< Front-left IR receiver GPIO port. */
#define PIN_IR_FRONT_LEFT_PIN          GPIO_PIN_5 /**< Front-left IR receiver GPIO pin. */
#define PIN_IR_FRONT_LEFT_ADC_CHANNEL  ADC_CHANNEL_5 /**< Front-left IR receiver ADC1 channel. */

#define PIN_IR_FRONT_RIGHT_PORT        GPIOA /**< Front-right IR receiver GPIO port. */
#define PIN_IR_FRONT_RIGHT_PIN         GPIO_PIN_4 /**< Front-right IR receiver GPIO pin. */
#define PIN_IR_FRONT_RIGHT_ADC_CHANNEL ADC_CHANNEL_4 /**< Front-right IR receiver ADC1 channel. */

#define PIN_IR_SIDE_LEFT_PORT          GPIOA /**< Left-side IR receiver GPIO port. */
#define PIN_IR_SIDE_LEFT_PIN           GPIO_PIN_3 /**< Left-side IR receiver GPIO pin. */
#define PIN_IR_SIDE_LEFT_ADC_CHANNEL   ADC_CHANNEL_3 /**< Left-side IR receiver ADC1 channel. */

#define PIN_IR_SIDE_RIGHT_PORT         GPIOA /**< Right-side IR receiver GPIO port. */
#define PIN_IR_SIDE_RIGHT_PIN          GPIO_PIN_2 /**< Right-side IR receiver GPIO pin. */
#define PIN_IR_SIDE_RIGHT_ADC_CHANNEL  ADC_CHANNEL_2 /**< Right-side IR receiver ADC1 channel. */

#define PIN_IR_DIAG_LEFT_PORT          GPIOA /**< Left diagonal IR receiver GPIO port. */
#define PIN_IR_DIAG_LEFT_PIN           GPIO_PIN_7 /**< Left diagonal IR receiver GPIO pin. */
#define PIN_IR_DIAG_LEFT_ADC_CHANNEL   ADC_CHANNEL_7 /**< Left diagonal IR receiver ADC1 channel. */

#define PIN_IR_DIAG_RIGHT_PORT         GPIOA /**< Right diagonal IR receiver GPIO port. */
#define PIN_IR_DIAG_RIGHT_PIN          GPIO_PIN_6 /**< Right diagonal IR receiver GPIO pin. */
#define PIN_IR_DIAG_RIGHT_ADC_CHANNEL  ADC_CHANNEL_6 /**< Right diagonal IR receiver ADC1 channel. */

/* ========================================================================== */
/* MPU6500 SPI interface                                                       */
/* ========================================================================== */

#define PIN_IMU_SCK_PORT               GPIOB /**< MPU6500 SPI clock GPIO port. */
#define PIN_IMU_SCK_PIN                GPIO_PIN_10 /**< MPU6500 SPI clock GPIO pin. */
#define PIN_IMU_SCK_AF                 GPIO_AF5_SPI2 /**< PB10 alternate function: SPI2_SCK. */
#define PIN_IMU_MISO_PORT              GPIOC /**< MPU6500 SPI MISO GPIO port. */
#define PIN_IMU_MISO_PIN               GPIO_PIN_2 /**< MPU6500 SPI MISO GPIO pin. */
#define PIN_IMU_MISO_AF                GPIO_AF5_SPI2 /**< PC2 alternate function: SPI2_MISO. */
#define PIN_IMU_MOSI_PORT              GPIOC /**< MPU6500 SPI MOSI GPIO port. */
#define PIN_IMU_MOSI_PIN               GPIO_PIN_3 /**< MPU6500 SPI MOSI GPIO pin. */
#define PIN_IMU_MOSI_AF                GPIO_AF5_SPI2 /**< PC3 alternate function: SPI2_MOSI. */
#define PIN_IMU_CS_PORT                GPIOA /**< MPU6500 chip-select GPIO port. */
#define PIN_IMU_CS_PIN                 GPIO_PIN_10 /**< MPU6500 chip-select GPIO pin. */

/* ========================================================================== */
/* SSD1306 OLED: I2C1                                                         */
/* ========================================================================== */

#define PIN_OLED_SCL_PORT              GPIOB /**< OLED I2C clock GPIO port. */
#define PIN_OLED_SCL_PIN               GPIO_PIN_8 /**< OLED I2C clock GPIO pin. */
#define PIN_OLED_SCL_AF                GPIO_AF4_I2C1 /**< PB8 alternate function: I2C1_SCL. */
#define PIN_OLED_SDA_PORT              GPIOB /**< OLED I2C data GPIO port. */
#define PIN_OLED_SDA_PIN               GPIO_PIN_9 /**< OLED I2C data GPIO pin. */
#define PIN_OLED_SDA_AF                GPIO_AF4_I2C1 /**< PB9 alternate function: I2C1_SDA. */
#define PIN_OLED_I2C_ADDRESS           (0x3CU) /**< SSD1306 seven-bit I2C address. */

/* ========================================================================== */
/* Battery monitor: ADC1                                                      */
/* ========================================================================== */

#define PIN_BATTERY_MONITOR_PORT       GPIOB /**< Battery-divider ADC GPIO port. */
#define PIN_BATTERY_MONITOR_PIN        GPIO_PIN_0 /**< Battery-divider ADC GPIO pin. */
#define PIN_BATTERY_MONITOR_ADC_CHANNEL ADC_CHANNEL_8 /**< Battery-divider ADC1 channel. */

/* ========================================================================== */
/* User interface                                                              */
/* ========================================================================== */

#define PIN_DIP_1_PORT                 GPIOA /**< DIP switch 1 GPIO port. */
#define PIN_DIP_1_PIN                  GPIO_PIN_12 /**< DIP switch 1 GPIO pin. */
#define PIN_DIP_2_PORT                 GPIOB /**< DIP switch 2 GPIO port. */
#define PIN_DIP_2_PIN                  GPIO_PIN_2 /**< DIP switch 2 GPIO pin. */
#define PIN_DIP_3_PORT                 GPIOC /**< DIP switch 3 GPIO port. */
#define PIN_DIP_3_PIN                  GPIO_PIN_14 /**< DIP switch 3 GPIO pin. */
#define PIN_DIP_4_PORT                 GPIOC /**< DIP switch 4 GPIO port. */
#define PIN_DIP_4_PIN                  GPIO_PIN_0 /**< DIP switch 4 GPIO pin. */

#define PIN_BUTTON_PORT                GPIOC /**< User/test button GPIO port. */
#define PIN_BUTTON_PIN                 GPIO_PIN_15 /**< User/test button GPIO pin. */

#define PIN_LED_PORT                   GPIOC /**< Onboard status LED GPIO port. */
#define PIN_LED_PIN                    GPIO_PIN_13 /**< Onboard status LED GPIO pin. */

/* ========================================================================== */
/* Buzzer: TIM3                                                               */
/* ========================================================================== */

#define PIN_BUZZER_TIMER               TIM3 /**< Buzzer timer instance. */
#define PIN_BUZZER_CHANNEL             TIM_CHANNEL_4 /**< Buzzer timer output channel. */
#define PIN_BUZZER_PORT                GPIOB /**< Buzzer GPIO port. */
#define PIN_BUZZER_PIN                 GPIO_PIN_1 /**< Buzzer GPIO pin. */
#define PIN_BUZZER_AF                  GPIO_AF2_TIM3 /**< PB1 alternate function: TIM3_CH4. */

#ifdef __cplusplus
}
#endif

#endif /* PINS_H */



/* ===================================================================== *
 * SECTION 15 —  COMPLETE PIN CONFLICT MATRIX
 *
 * Pin   | Signal      | Peripheral | Notes
 * ------|-------------|------------|----------------------------------
 * PA0   | L_EN_A      | TIM2_CH1   | Left Encoder Channel A Input
 * PA1   | L_EN_B      | TIM2_CH2   | Left Encoder Channel B Input
 * PA2   | RS_RE       | ADC1_IN2   | Right Side Phototransistor (Analog)
 * PA3   | LS_RE       | ADC1_IN3   | Left Side Phototransistor (Analog)
 * PA4   | RF_RE       | ADC1_IN4   | Right Front Phototransistor (Analog)
 * PA5   | LF_RE       | ADC1_IN5   | Left Front Phototransistor (Analog)
 * PA6   | R_AnRE      | ADC1_IN6   | Right Angle Phototransistor (Analog)
 * PA7   | L_AnRE      | ADC1_IN7   | Left Angle Phototransistor (Analog)
 * PA8   | LM_PWM      | TIM1_CH1   | Left Motor PWM Out (AF01)
 * PA9   | RM_PWM      | TIM1_CH2   | Right Motor PWM Out (AF01)
 * PA10  | R_ANGLE_EM  | GPIO OUT   | Right Angle IR Emitter Switching Node
 * PA11  | L_ANGLE_EM  | GPIO OUT   | Left Angle IR Emitter Switching Node
 * PA12  | DIP1        | GPIO IN PU | Hardware Mode Configuration Bit 0
 * PA13  | SWDIO       | SYS SWD    | Serial Wire In/Out (Keep Reserved)
 * PA14  | SWDCLK      | SYS SWD    | Serial Wire Clock Input (Keep Reserved)
 * PA15  | RF_EM       | GPIO OUT   | Right Front Emitter (Requires JTAG Switch Disable)
 * PB0   | VOLTMETER   | ADC1_IN8   | Battery Divider Node Voltage Check (Analog)
 * PB1   | BUZZER      | GPIO OUT   | Transistor Audio Indication Signal Out
 * PB2   | DIP2        | GPIO IN PU | Hardware Mode Configuration Bit 1
 * PB3   | LF_EM       | GPIO OUT   | Left Front Emitter (Requires JTAG Switch Disable)
 * PB4   | R_SIDE_EM   | GPIO OUT   | Right Side IR Emitter Switching Node
 * PB5   | L_SIDE_EM   | GPIO OUT   | Left Side IR Emitter Switching Node
 * PB6   | R_EN_A      | TIM4_CH1   | Right Encoder Channel A Input (AF02)
 * PB7   | R_EN_B      | TIM4_CH2   | Right Encoder Channel B Input (AF02)
 * PB8   | G_SCL       | I2C1_SCL   | IMU I2C Bus Clock Output Line (AF04, Pullup Required)
 * PB9   | G_SDA       | I2C1_SDA   | IMU I2C Bus Serial Data Line  (AF04, Pullup Required)
 * PB10  | STBY        | GPIO OUT   | Motor Driver Chip Standby Switch H->Run L->Sleep
 * PB12  | LM_IN1      | GPIO OUT   | H-Bridge Left Motor Input Control Bit 0
 * PB13  | LM_IN2      | GPIO OUT   | H-Bridge Left Motor Input Control Bit 1
 * PB14  | RM_IN1      | GPIO OUT   | H-Bridge Right Motor Input Control Bit 0
 * PB15  | RM_IN2      | GPIO OUT   | H-Bridge Right Motor Input Control Bit 1
 * PC13  | DIP4 / LED  | MULTI OUT  | Shared Node. Read Switch Boot Init -> Shift to Status LED
 * PC14  | DIP3        | GPIO IN PU | Hardware Mode Configuration Bit 2
 * PC15  | BTN         | GPIO IN PU | Pushbutton UI Signal Input line (Active Low)
 * ===================================================================== */

