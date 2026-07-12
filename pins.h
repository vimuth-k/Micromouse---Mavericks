/**
 * @file    pins.h
 * @brief   MicroMaze 3 · STM32F411CEU6 Black Pill · Pin Definitions
 * * Single source of truth for all physical hardware connections. This file 
 * contains exclusively register mappings, peripheral associations, constant 
 * configurations, and documentation. No driver logic or helper functions are 
 * permitted here.
 *
 * @note All configurations are verified for STM32CubeMX compatibility.
 */

#ifndef PINS_H
#define PINS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ===================================================================== *
 * SECTION 1  —  MOTOR PWM (TIM1, APB2 = 100 MHz)
 * Configuration: Prescaler = 0, ARR = 4999 -> 20 kHz PWM Frequency
 * ===================================================================== */

/** @name Left Motor PWM Definitions */
/**@{*/
#define MOTOR_L_PWM_PORT        GPIOA            /**< GPIO Port for Left Motor PWM */
#define MOTOR_L_PWM_PIN         GPIO_PIN_8       /**< GPIO Pin for Left Motor PWM */
#define MOTOR_L_PWM_AF          GPIO_AF1_TIM1    /**< Alternate Function mapping to TIM1 */
#define MOTOR_L_TIM             TIM1             /**< Timer instance for Left Motor */
#define MOTOR_L_TIM_CHANNEL     TIM_CHANNEL_1    /**< Timer Channel for Left Motor */
/**@}*/

/** @name Right Motor PWM Definitions */
/**@{*/
#define MOTOR_R_PWM_PORT        GPIOA            /**< GPIO Port for Right Motor PWM */
#define MOTOR_R_PWM_PIN         GPIO_PIN_9       /**< GPIO Pin for Right Motor PWM */
#define MOTOR_R_PWM_AF          GPIO_AF1_TIM1    /**< Alternate Function mapping to TIM1 */
#define MOTOR_R_TIM             TIM1             /**< Timer instance for Right Motor */
#define MOTOR_R_TIM_CHANNEL     TIM_CHANNEL_2    /**< Timer Channel for Right Motor */
/**@}*/

/** @name Shared Motor Timer Constants */
/**@{*/
#define MOTOR_TIM_INSTANCE      TIM1             /**< Shared Timer peripheral instance */
#define MOTOR_TIM_PRESCALER     0U               /**< Clock prescaler value */
#define MOTOR_TIM_PERIOD        4999U            /**< Auto-reload value for 20 kHz frequency */
/**@}*/

/* ===================================================================== *
 * SECTION 2  —  MOTOR DIRECTION PINS (H-Bridge Interface)
 * ===================================================================== */

/** @name Left Motor Direction Control */
/**@{*/
#define MOTOR_L_IN1_PORT        GPIOB            /**< GPIO Port for Left Motor AIN1 */
#define MOTOR_L_IN1_PIN         GPIO_PIN_12      /**< GPIO Pin for Left Motor AIN1 */
#define MOTOR_L_IN2_PORT        GPIOB            /**< GPIO Port for Left Motor AIN2 */
#define MOTOR_L_IN2_PIN         GPIO_PIN_13      /**< GPIO Pin for Left Motor AIN2 */
/**@}*/

/** @name Right Motor Direction Control */
/**@{*/
#define MOTOR_R_IN1_PORT        GPIOB            /**< GPIO Port for Right Motor BIN1 */
#define MOTOR_R_IN1_PIN         GPIO_PIN_14      /**< GPIO Pin for Right Motor BIN1 */
#define MOTOR_R_IN2_PORT        GPIOB            /**< GPIO Port for Right Motor BIN2 */
#define MOTOR_R_IN2_PIN         GPIO_PIN_15      /**< GPIO Pin for Right Motor BIN2 */
/**@}*/

/* ===================================================================== *
 * SECTION 3  —  MOTOR STANDBY PIN
 * ===================================================================== */

#define MOTOR_STBY_PORT         GPIOB            /**< GPIO Port for Motor Driver Standby */
#define MOTOR_STBY_PIN          GPIO_PIN_10      /**< GPIO Pin for Motor Driver Standby */

/* ===================================================================== *
 * SECTION 4  —  ENCODERS (TIM2 32-bit / TIM4 16-bit)
 * ===================================================================== */

/** @name Left Encoder Definitions (32-bit Quadrature) */
/**@{*/
#define ENC_L_A_PORT            GPIOA            /**< GPIO Port for Left Encoder Channel A */
#define ENC_L_A_PIN             GPIO_PIN_0       /**< GPIO Pin for Left Encoder Channel A */
#define ENC_L_A_AF              GPIO_AF1_TIM2    /**< Alternate function mapping to TIM2 */
#define ENC_L_B_PORT            GPIOA            /**< GPIO Port for Left Encoder Channel B */
#define ENC_L_B_PIN             GPIO_PIN_1       /**< GPIO Pin for Left Encoder Channel B */
#define ENC_L_B_AF              GPIO_AF1_TIM2    /**< Alternate function mapping to TIM2 */
#define ENC_L_TIM               TIM2             /**< Timer instance used for Left Encoder */
/**@}*/

/** @name Right Encoder Definitions (16-bit Quadrature + Overflow Track) */
/**@{*/
#define ENC_R_A_PORT            GPIOB            /**< GPIO Port for Right Encoder Channel A */
#define ENC_R_A_PIN             GPIO_PIN_6       /**< GPIO Pin for Right Encoder Channel A */
#define ENC_R_A_AF              GPIO_AF2_TIM4    /**< Alternate function mapping to TIM4 */
#define ENC_R_B_PORT            GPIOB            /**< GPIO Port for Right Encoder Channel B */
#define ENC_R_B_PIN             GPIO_PIN_7       /**< GPIO Pin for Right Encoder Channel B */
#define ENC_R_B_AF              GPIO_AF2_TIM4    /**< Alternate function mapping to TIM4 */
#define ENC_R_TIM               TIM4             /**< Timer instance used for Right Encoder */
#define ENC_R_MAX_COUNT         65535U           /**< Maximum counter limit before overflow */
/**@}*/

/* ===================================================================== *
 * SECTION 5  —  CONTROL LOOP TIMER (Internal Ticker)
 * ===================================================================== */

#define CTRL_TIM_INSTANCE       TIM5             /**< Timer peripheral for PID loop ticker */
#define CTRL_TIM_PRESCALER      99U              /**< Prescaler mapping down to 1 MHz clock frequency */
#define CTRL_TIM_PERIOD         999U             /**< Period configuration creating a 1 kHz interrupt */
#define CTRL_TIM_PRIORITY       1U               /**< Preemption priority configuration for NVIC */

/* ===================================================================== *
 * SECTION 6  —  IR EMITTERS (Active High Gate Controls)
 * ===================================================================== */

#define IR_EMIT_R_ANGLE_PORT    GPIOA            /**< GPIO Port for Right Angle Emitter */
#define IR_EMIT_R_ANGLE_PIN     GPIO_PIN_10      /**< GPIO Pin for Right Angle Emitter */

#define IR_EMIT_L_ANGLE_PORT    GPIOA            /**< GPIO Port for Left Angle Emitter */
#define IR_EMIT_L_ANGLE_PIN     GPIO_PIN_11      /**< GPIO Pin for Left Angle Emitter */

#define IR_EMIT_RF_PORT         GPIOA            /**< GPIO Port for Right Front Emitter (JTAG Override) */
#define IR_EMIT_RF_PIN          GPIO_PIN_15      /**< GPIO Pin for Right Front Emitter (JTAG Override) */

#define IR_EMIT_LF_PORT         GPIOB            /**< GPIO Port for Left Front Emitter (JTAG Override) */
#define IR_EMIT_LF_PIN          GPIO_PIN_3       /**< GPIO Pin for Left Front Emitter (JTAG Override) */

#define IR_EMIT_R_SIDE_PORT     GPIOB            /**< GPIO Port for Right Side Emitter */
#define IR_EMIT_R_SIDE_PIN      GPIO_PIN_4       /**< GPIO Pin for Right Side Emitter */

#define IR_EMIT_L_SIDE_PORT     GPIOB            /**< GPIO Port for Left Side Emitter */
#define IR_EMIT_L_SIDE_PIN      GPIO_PIN_5       /**< GPIO Pin for Left Side Emitter */

/* ===================================================================== *
 * SECTION 7  —  IR RECEIVERS (ADC1 Multi-channel DMA Scan)
 * ===================================================================== */

/** @name DMA Circular Buffer Index Mapping */
/**@{*/
#define IR_IDX_RS               0U               /**< Buffer Index for Right Side Sensor */
#define IR_IDX_LS               1U               /**< Buffer Index for Left Side Sensor */
#define IR_IDX_RF               2U               /**< Buffer Index for Right Front Sensor */
#define IR_IDX_LF               3U               /**< Buffer Index for Left Front Sensor */
#define IR_IDX_R_ANG            4U               /**< Buffer Index for Right Angle Sensor */
#define IR_IDX_L_ANG            5U               /**< Buffer Index for Left Angle Sensor */
#define IR_IDX_BATT             6U               /**< Buffer Index for Battery Voltage Sensor */
#define IR_ADC_BUF_LEN          7U               /**< Total DMA Scan Conversion Length */
/**@}*/

/** @name Receiver Pin Hardware Maps */
/**@{*/
#define IR_RECV_RS_PORT         GPIOA            /**< GPIO Port for Right Side Phototransistor */
#define IR_RECV_RS_PIN          GPIO_PIN_2       /**< GPIO Pin for Right Side Phototransistor */
#define IR_RECV_RS_ADC_CH       ADC_CHANNEL_2    /**< ADC Channel for Right Side Sensor */
#define IR_RECV_RS_ADC_RANK     1U               /**< Scan Sequence Rank for Right Side Sensor */

#define IR_RECV_LS_PORT         GPIOA            /**< GPIO Port for Left Side Phototransistor */
#define IR_RECV_LS_PIN          GPIO_PIN_3       /**< GPIO Pin for Left Side Phototransistor */
#define IR_RECV_LS_ADC_CH       ADC_CHANNEL_3    /**< ADC Channel for Left Side Sensor */
#define IR_RECV_LS_ADC_RANK     2U               /**< Scan Sequence Rank for Left Side Sensor */

#define IR_RECV_RF_PORT         GPIOA            /**< GPIO Port for Right Front Phototransistor */
#define IR_RECV_RF_PIN          GPIO_PIN_4       /**< GPIO Pin for Right Front Phototransistor */
#define IR_RECV_RF_ADC_CH       ADC_CHANNEL_4    /**< ADC Channel for Right Front Sensor */
#define IR_RECV_RF_ADC_RANK     3U               /**< Scan Sequence Rank for Right Front Sensor */

#define IR_RECV_LF_PORT         GPIOA            /**< GPIO Port for Left Front Phototransistor */
#define IR_RECV_LF_PIN          GPIO_PIN_5       /**< GPIO Pin for Left Front Phototransistor */
#define IR_RECV_LF_ADC_CH       ADC_CHANNEL_5    /**< ADC Channel for Left Front Sensor */
#define IR_RECV_LF_ADC_RANK     4U               /**< Scan Sequence Rank for Left Front Sensor */

#define IR_RECV_R_ANG_PORT      GPIOA            /**< GPIO Port for Right Angle Phototransistor */
#define IR_RECV_R_ANG_PIN       GPIO_PIN_6       /**< GPIO Pin for Right Angle Phototransistor */
#define IR_RECV_R_ANG_ADC_CH    ADC_CHANNEL_6    /**< ADC Channel for Right Angle Sensor */
#define IR_RECV_R_ANG_ADC_RANK  5U               /**< Scan Sequence Rank for Right Angle Sensor */

#define IR_RECV_L_ANG_PORT      GPIOA            /**< GPIO Port for Left Angle Phototransistor */
#define IR_RECV_L_ANG_PIN       GPIO_PIN_7       /**< GPIO Pin for Left Angle Phototransistor */
#define IR_RECV_L_ANG_ADC_CH    ADC_CHANNEL_7    /**< ADC Channel for Left Angle Sensor */
#define IR_RECV_L_ANG_ADC_RANK  6U               /**< Scan Sequence Rank for Left Angle Sensor */

#define IR_ADC_INSTANCE         ADC1             /**< ADC Peripheral Instance for Scanning Array */
/**@}*/

/* ===================================================================== *
 * SECTION 8  —  BATTERY VOLTAGE ADC
 * ===================================================================== */

#define BATT_ADC_PORT           GPIOB            /**< GPIO Port for Voltage Divider Node */
#define BATT_ADC_PIN            GPIO_PIN_0       /**< GPIO Pin for Voltage Divider Node */
#define BATT_ADC_CH             ADC_CHANNEL_8    /**< ADC Channel mapped to Divider Node */
#define BATT_ADC_RANK           7U               /**< Final Scan Sequence Rank for Battery Check */
#define BATT_ADC_INSTANCE       ADC1             /**< Shared ADC Peripheral Instance */

/* ===================================================================== *
 * SECTION 9  —  IMU / GYROSCOPE (MPU6500 Fast Mode I2C)
 * ===================================================================== */

#define IMU_SCL_PORT            GPIOB            /**< GPIO Port for I2C Serial Clock */
#define IMU_SCL_PIN             GPIO_PIN_8       /**< GPIO Pin for I2C Serial Clock */
#define IMU_SCL_AF              GPIO_AF4_I2C1    /**< Alternate Function mapping to I2C1 */

#define IMU_SDA_PORT            GPIOB            /**< GPIO Port for I2C Serial Data */
#define IMU_SDA_PIN             GPIO_PIN_9       /**< GPIO Pin for I2C Serial Data */
#define IMU_SDA_AF              GPIO_AF4_I2C1    /**< Alternate Function mapping to I2C1 */

#define IMU_I2C_INSTANCE        I2C1             /**< I2C Hardware Instance */
#define MPU6500_I2C_ADDR_8BIT   (0x68U << 1U)    /**< 8-bit Shifted Format for HAL Drivers */

/* ===================================================================== *
 * SECTION 10 —  DIP SWITCHES (User UI Configuration Inputs)
 * ===================================================================== */

#define DIP1_PORT               GPIOA            /**< GPIO Port for DIP Switch Position 1 */
#define DIP1_PIN                GPIO_PIN_12      /**< GPIO Pin for DIP Switch Position 1 */

#define DIP2_PORT               GPIOB            /**< GPIO Port for DIP Switch Position 2 */
#define DIP2_PIN                GPIO_PIN_2       /**< GPIO Pin for DIP Switch Position 2 */

#define DIP3_PORT               GPIOC            /**< GPIO Port for DIP Switch Position 3 */
#define DIP3_PIN                GPIO_PIN_14      /**< GPIO Pin for DIP Switch Position 3 */

#define DIP4_PORT               GPIOC            /**< GPIO Port for DIP Switch Position 4 (Shared with Onboard LED) */
#define DIP4_PIN                GPIO_PIN_13      /**< GPIO Pin for DIP Switch Position 4 (Shared with Onboard LED) */

/* ===================================================================== *
 * SECTION 11 —  USER BUTTON
 * ===================================================================== */

#define BTN_PORT                GPIOC            /**< GPIO Port for Onboard Test/User Button */
#define BTN_PIN                 GPIO_PIN_15      /**< GPIO Pin for Onboard Test/User Button */

/* ===================================================================== *
 * SECTION 12 —  BUZZER
 * ===================================================================== */

#define BUZZER_PORT             GPIOB            /**< GPIO Port for Audio Feedback Node */
#define BUZZER_PIN              GPIO_PIN_1       /**< GPIO Pin for Audio Feedback Node */

/* ===================================================================== *
 * SECTION 13 —  STATUS LED
 * ===================================================================== */

#define LED_PORT                GPIOC            /**< GPIO Port for Status Indicator LED */
#define LED_PIN                 GPIO_PIN_13      /**< GPIO Pin for Status Indicator LED */

/* ===================================================================== *
 * SECTION 14 —  RESERVED SYSTEM PINS
 * ===================================================================== */

#define SYS_SWDIO_PORT          GPIOA            /**< Fixed Port for Serial Wire Data Input Output */
#define SYS_SWDIO_PIN           GPIO_PIN_13      /**< Fixed Pin for Serial Wire Data Input Output */

#define SYS_SWCLK_PORT          GPIOA            /**< Fixed Port for Serial Wire Clock */
#define SYS_SWCLK_PIN           GPIO_PIN_14      /**< Fixed Pin for Serial Wire Clock */


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

#ifdef __cplusplus
}
#endif

#endif /* PINS_H */
