#ifndef PINS_H
#define PINS_H

/*
 * ================================================================
 *  MICROMOUSE  ·  STM32F411CEU6 Black Pill  ·  PINS.H
 *  Single source of truth for every GPIO / peripheral pin.
 *  Change pin numbers HERE only — all other files use these macros.
 * ================================================================
 *
 *  COMPONENTS
 *  ─────────────────────────────────────────────────────────────
 *  Motors  : 2× 6V 750RPM DC with quadrature encoders
 *  Driver  : TB6612FNG  (dual H-bridge)
 *  IMU     : MPU6500    (SPI2)
 *  IR emit : 6× SFH4545 via AO3400A N-MOSFET gates
 *  IR recv : 6× TEFT4300 phototransistors → ADC1 DMA scan
 *  Display : SSD1306 128×64 OLED (I2C1, optional)
 *  DIP SW  : 4-position
 *  Wheel   : 34 mm diameter rubber
 * ================================================================
 *
 *  CONFLICT RESOLUTION NOTES
 *  ─────────────────────────────────────────────────────────────
 *  PA9  = TIM1_CH2 (PWMB)  conflicts with USART1_TX
 *      → Use USB CDC for UART debug (no pin needed)
 *  PA5  = SPI1_SCK          conflicts with ADC1_IN5
 *      → Use SPI2 for MPU6500 (PB10/PC2/PC3)
 *  PA0  = TIM2_CH1 encoder  conflicts with WKUP button
 *      → Leave PA0 as encoder; use PB5 as run button
 * ================================================================
 */

#include "stm32f4xx_hal.h"

/* ── MOTOR PWM  (TIM1, APB2, 100 MHz) ──────────────────────────────── */
/* Left  motor speed  → TIM1 CH1 */
#define PIN_PWMA_PORT           GPIOA
#define PIN_PWMA_PIN            GPIO_PIN_8      /* PA8  TIM1_CH1          */
/* Right motor speed  → TIM1 CH2 */
#define PIN_PWMB_PORT           GPIOA
#define PIN_PWMB_PIN            GPIO_PIN_9      /* PA9  TIM1_CH2          */

/* ── TB6612FNG DIRECTION PINS ───────────────────────────────────────── */
/* Left motor (Channel A of TB6612) */
#define PIN_AIN1_PORT           GPIOB
#define PIN_AIN1_PIN            GPIO_PIN_2      /* PB2  GPIO OUT          */
#define PIN_AIN2_PORT           GPIOB
#define PIN_AIN2_PIN            GPIO_PIN_3      /* PB3  GPIO OUT          */
/* Right motor (Channel B of TB6612) */
#define PIN_BIN1_PORT           GPIOB
#define PIN_BIN1_PIN            GPIO_PIN_12     /* PB12 GPIO OUT          */
#define PIN_BIN2_PORT           GPIOB
#define PIN_BIN2_PIN            GPIO_PIN_11     /* PB11 GPIO OUT          */
/* Standby pin (shared, active HIGH) */
#define PIN_STBY_PORT           GPIOB
#define PIN_STBY_PIN            GPIO_PIN_1      /* PB1  GPIO OUT          */

/* ── ENCODERS ───────────────────────────────────────────────────────── */
/* Left encoder   → TIM2 (32-bit) encoder mode */
#define PIN_ENC_LA_PORT         GPIOA
#define PIN_ENC_LA_PIN          GPIO_PIN_0      /* PA0  TIM2_CH1          */
#define PIN_ENC_LB_PORT         GPIOA
#define PIN_ENC_LB_PIN          GPIO_PIN_1      /* PA1  TIM2_CH2          */
/* Right encoder  → TIM4 (16-bit) encoder mode */
#define PIN_ENC_RA_PORT         GPIOB
#define PIN_ENC_RA_PIN          GPIO_PIN_6      /* PB6  TIM4_CH1          */
#define PIN_ENC_RB_PORT         GPIOB
#define PIN_ENC_RB_PIN          GPIO_PIN_7      /* PB7  TIM4_CH2          */

/* ── IR EMITTERS  (AO3400A MOSFET gate, 1 pin fires ALL 6 emitters) ── */
/* All 6 SFH4545 emitters are connected to MOSFETs driven from PB0.    */
/* Each MOSFET gate has a 100Ω series resistor.                         */
/* Each SFH4545 has a series resistor to limit peak current to ~80mA.  */
#define PIN_IR_EMIT_PORT        GPIOB
#define PIN_IR_EMIT_PIN         GPIO_PIN_0      /* PB0  GPIO OUT          */

/* ── IR RECEIVERS  (ADC1, 6-channel DMA circular scan) ─────────────── */
/* Receivers: TEFT4300 collector → 10kΩ pull-up to 3.3V → ADC pin     */
/* Rank order in CubeMX must match these indices exactly:               */
/*  Rank 1 → SENS_FL (front-left  diagonal)  PA2  ADC1_IN2             */
/*  Rank 2 → SENS_FR (front-right diagonal)  PA3  ADC1_IN3             */
/*  Rank 3 → SENS_L  (left side)             PA4  ADC1_IN4             */
/*  Rank 4 → SENS_R  (right side)            PA5  ADC1_IN5             */
/*  Rank 5 → SENS_DL (diagonal left)         PA6  ADC1_IN6             */
/*  Rank 6 → SENS_DR (diagonal right)        PA7  ADC1_IN7             */
#define PIN_ADC_FL_PORT         GPIOA
#define PIN_ADC_FL_PIN          GPIO_PIN_2      /* PA2  ADC1_IN2          */
#define PIN_ADC_FR_PORT         GPIOA
#define PIN_ADC_FR_PIN          GPIO_PIN_3      /* PA3  ADC1_IN3          */
#define PIN_ADC_L_PORT          GPIOA
#define PIN_ADC_L_PIN           GPIO_PIN_4      /* PA4  ADC1_IN4          */
#define PIN_ADC_R_PORT          GPIOA
#define PIN_ADC_R_PIN           GPIO_PIN_5      /* PA5  ADC1_IN5          */
#define PIN_ADC_DL_PORT         GPIOA
#define PIN_ADC_DL_PIN          GPIO_PIN_6      /* PA6  ADC1_IN6          */
#define PIN_ADC_DR_PORT         GPIOA
#define PIN_ADC_DR_PIN          GPIO_PIN_7      /* PA7  ADC1_IN7          */

/* ── MPU6500 IMU  (SPI2) ────────────────────────────────────────────── */
#define PIN_SPI2_SCK_PORT       GPIOB
#define PIN_SPI2_SCK_PIN        GPIO_PIN_10     /* PB10 SPI2_SCK          */
#define PIN_SPI2_MISO_PORT      GPIOC
#define PIN_SPI2_MISO_PIN       GPIO_PIN_2      /* PC2  SPI2_MISO         */
#define PIN_SPI2_MOSI_PORT      GPIOC
#define PIN_SPI2_MOSI_PIN       GPIO_PIN_3      /* PC3  SPI2_MOSI         */
#define PIN_MPU_CS_PORT         GPIOA
#define PIN_MPU_CS_PIN          GPIO_PIN_10     /* PA10 GPIO OUT CS       */

/* ── OLED SSD1306  (I2C1) ───────────────────────────────────────────── */
#define PIN_I2C1_SCL_PORT       GPIOB
#define PIN_I2C1_SCL_PIN        GPIO_PIN_8      /* PB8  I2C1_SCL          */
#define PIN_I2C1_SDA_PORT       GPIOB
#define PIN_I2C1_SDA_PIN        GPIO_PIN_9      /* PB9  I2C1_SDA          */

/* ── DIP SWITCHES  (active LOW, internal pull-up) ───────────────────── */
#define PIN_SW0_PORT            GPIOC
#define PIN_SW0_PIN             GPIO_PIN_14     /* PC14 GPIO IN           */
#define PIN_SW1_PORT            GPIOA
#define PIN_SW1_PIN             GPIO_PIN_15     /* PA15 GPIO IN           */
#define PIN_SW2_PORT            GPIOB
#define PIN_SW2_PIN             GPIO_PIN_4      /* PB4  GPIO IN           */
#define PIN_SW3_PORT            GPIOB
#define PIN_SW3_PIN             GPIO_PIN_5      /* PB5  GPIO IN  ← SW3   */

/* ── RUN BUTTON  (active LOW, internal pull-up) ─────────────────────── */
/* Separate from DIP switches. Press to trigger a run. */
#define PIN_BTN_PORT            GPIOC
#define PIN_BTN_PIN             GPIO_PIN_15     /* PC15 GPIO IN           */

/* ── STATUS LED  (onboard Black Pill LED, active LOW) ───────────────── */
#define PIN_LED_PORT            GPIOC
#define PIN_LED_PIN             GPIO_PIN_13     /* PC13 GPIO OUT          */

/* ── BUZZER  (active HIGH, via NPN transistor or MOSFET) ────────────── */
#define PIN_BUZZER_PORT         GPIOB
#define PIN_BUZZER_PIN          GPIO_PIN_13     /* PB13 GPIO OUT          */

/* ── BATTERY ADC  (voltage divider: Vbat÷3 → ADC) ─────────────────── */
/* Use a 2:1 voltage divider (2× 10kΩ): Vbat/2 → PB0 */
/* PB0 is already used for IR_EMIT — move BATT_ADC to PC0 instead       */
#define PIN_BATT_ADC_PORT       GPIOC
#define PIN_BATT_ADC_PIN        GPIO_PIN_0      /* PC0  ADC1_IN10 (alt)   */
/* NOTE: Add this as Rank 7 in the ADC DMA scan, or use ADC2 separately */

/* ── CONVENIENCE MACROS ─────────────────────────────────────────────── */
#define LED_ON()        HAL_GPIO_WritePin(PIN_LED_PORT,    PIN_LED_PIN,    GPIO_PIN_RESET)
#define LED_OFF()       HAL_GPIO_WritePin(PIN_LED_PORT,    PIN_LED_PIN,    GPIO_PIN_SET)
#define LED_TOGGLE()    HAL_GPIO_TogglePin(PIN_LED_PORT,   PIN_LED_PIN)

#define IR_EMIT_ON()    HAL_GPIO_WritePin(PIN_IR_EMIT_PORT, PIN_IR_EMIT_PIN, GPIO_PIN_SET)
#define IR_EMIT_OFF()   HAL_GPIO_WritePin(PIN_IR_EMIT_PORT, PIN_IR_EMIT_PIN, GPIO_PIN_RESET)

#define MPU_CS_LOW()    HAL_GPIO_WritePin(PIN_MPU_CS_PORT,  PIN_MPU_CS_PIN, GPIO_PIN_RESET)
#define MPU_CS_HIGH()   HAL_GPIO_WritePin(PIN_MPU_CS_PORT,  PIN_MPU_CS_PIN, GPIO_PIN_SET)

#define MOTOR_ENABLE()  HAL_GPIO_WritePin(PIN_STBY_PORT, PIN_STBY_PIN, GPIO_PIN_SET)
#define MOTOR_DISABLE() HAL_GPIO_WritePin(PIN_STBY_PORT, PIN_STBY_PIN, GPIO_PIN_RESET)

#define BUZZER_ON()     HAL_GPIO_WritePin(PIN_BUZZER_PORT, PIN_BUZZER_PIN, GPIO_PIN_SET)
#define BUZZER_OFF()    HAL_GPIO_WritePin(PIN_BUZZER_PORT, PIN_BUZZER_PIN, GPIO_PIN_RESET)

#define BTN_PRESSED()   (HAL_GPIO_ReadPin(PIN_BTN_PORT, PIN_BTN_PIN) == GPIO_PIN_RESET)

static inline uint8_t READ_DIP_SWITCHES(void)
{
    uint8_t v = 0;
    if (HAL_GPIO_ReadPin(PIN_SW0_PORT, PIN_SW0_PIN) == GPIO_PIN_RESET) v |= (1u << 0);
    if (HAL_GPIO_ReadPin(PIN_SW1_PORT, PIN_SW1_PIN) == GPIO_PIN_RESET) v |= (1u << 1);
    if (HAL_GPIO_ReadPin(PIN_SW2_PORT, PIN_SW2_PIN) == GPIO_PIN_RESET) v |= (1u << 2);
    if (HAL_GPIO_ReadPin(PIN_SW3_PORT, PIN_SW3_PIN) == GPIO_PIN_RESET) v |= (1u << 3);
    return v;
}

#endif /* PINS_H */
