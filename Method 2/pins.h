/**
 * @file    pins.h
 * @brief   MicroMaze 3  ·  STM32F411CEU6 Black Pill  ·  Pin Definitions
 *
 * Single source of truth for every hardware connection.
 * ─────────────────────────────────────────────────────────────────────
 * RULES:
 *   • One physical pin  → one PORT macro + one PIN macro.
 *   • No HAL_xxx() calls, no peripheral init, no structs.
 *   • All timer/ADC channel numbers derived from the same pin macros.
 *   • Every group is conflict-checked and CubeMX-compatible.
 *   • Helper macros (SET/RESET/TOGGLE/READ) follow each GPIO group.
 * ─────────────────────────────────────────────────────────────────────
 * PERIPHERAL SUMMARY  (rev 2 — post pin-remap, finalized)
 * ─────────────────────────────────────────────────────────────────────
 *  Peripheral   Timer/Bus   Pins
 *  ──────────── ─────────── ─────────────────────────────────────────
 *  Left  motor  TIM1_CH1    PA8   (PWM)
 *  Right motor  TIM1_CH2    PA9   (PWM)
 *  Left  enc    TIM2        PA0 / PA1  (CH1 / CH2, 32-bit)
 *  Right enc    TIM3        PB4 / PB5  (CH1 / CH2, 16-bit + OVF ISR)
 *  Control loop TIM5        (1 kHz interrupt, no pin)
 *  IR receivers ADC1 DMA    PA2–PA7, PB0  (7-channel scan, ranks 1–7)
 *  IMU MPU6500  I2C1        PB8 (SCL) / PB9 (SDA)
 *  IMU INT      EXTI IN     PC13  (wired, not yet consumed by imu.c)
 *  IR emitters  GPIO OUT    6 independent: PA10,PA11,PA15,PB1,PB2,PB3
 *  Motor dir    GPIO OUT    PB12,PB13 (left) / PB14,PB15 (right)
 *  Motor STBY   GPIO OUT    PB7
 *  DIP switches GPIO IN PU  PA12, PB6, PB10, PC14
 *  Test button  GPIO IN PU  PC15
 *
 *  Onboard LED and buzzer are NOT used by this firmware — PC13 is
 *  needed for MPU6500_INT and every remaining GPIO is spoken for.
 * ─────────────────────────────────────────────────────────────────────
 */

#ifndef PINS_H
#define PINS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"   /* GPIO_PIN_x, GPIOx, needed for helper macros */

/* ===================================================================== *
 *  SECTION 1  —  MOTOR PWM  (TIM1, APB2 = 100 MHz)
 *
 *  CubeMX:  TIM1 → PWM Generation CH1 + CH2
 *           Prescaler = 0,  ARR = 4999  → 20 kHz
 *           CH1 Mode = PWM1, CH2 Mode = PWM1, Preload ENABLE
 *           PA8 alternate function AF01 (TIM1_CH1)
 *           PA9 alternate function AF01 (TIM1_CH2)
 * ===================================================================== */

/** Left motor PWM — PA8 — TIM1_CH1 */
#define MOTOR_L_PWM_PORT        GPIOA
#define MOTOR_L_PWM_PIN         GPIO_PIN_8
#define MOTOR_L_PWM_AF          GPIO_AF1_TIM1
#define MOTOR_L_TIM             TIM1
#define MOTOR_L_TIM_CHANNEL     TIM_CHANNEL_1
#define MOTOR_L_CCR             (TIM1->CCR1)   /**< direct CCR write     */

/** Right motor PWM — PA9 — TIM1_CH2 */
#define MOTOR_R_PWM_PORT        GPIOA
#define MOTOR_R_PWM_PIN         GPIO_PIN_9
#define MOTOR_R_PWM_AF          GPIO_AF1_TIM1
#define MOTOR_R_TIM             TIM1
#define MOTOR_R_TIM_CHANNEL     TIM_CHANNEL_2
#define MOTOR_R_CCR             (TIM1->CCR2)   /**< direct CCR write     */

/** PWM timer shared settings */
#define MOTOR_TIM_INSTANCE      TIM1
#define MOTOR_TIM_PRESCALER     0U
#define MOTOR_TIM_PERIOD        4999U          /**< 100 MHz / 5000 = 20 kHz */
#define MOTOR_PWM_MAX           4999U
#define MOTOR_PWM_MIN           0U

/* ===================================================================== *
 *  SECTION 2  —  MOTOR DIRECTION  (TB6612FNG, GPIO OUT, no pull)
 *
 *  Truth table (per channel):
 *    xIN1 H, xIN2 L, PWM > 0  →  Forward
 *    xIN1 L, xIN2 H, PWM > 0  →  Reverse
 *    xIN1 H, xIN2 H,  any     →  Brake  (short-circuit stop)
 *    xIN1 L, xIN2 L,  any     →  Coast  (free-wheel)
 *
 *  CubeMX: PB12,PB13,PB14,PB15 → GPIO Output, No pull, Push-pull, Low speed
 * ===================================================================== */

/** Left motor direction bit 1 — PB12 — AIN1 */
#define MOTOR_L_IN1_PORT        GPIOB
#define MOTOR_L_IN1_PIN         GPIO_PIN_12

/** Left motor direction bit 2 — PB13 — AIN2 */
#define MOTOR_L_IN2_PORT        GPIOB
#define MOTOR_L_IN2_PIN         GPIO_PIN_13

/** Right motor direction bit 1 — PB14 — BIN1 */
#define MOTOR_R_IN1_PORT        GPIOB
#define MOTOR_R_IN1_PIN         GPIO_PIN_14

/** Right motor direction bit 2 — PB15 — BIN2 */
#define MOTOR_R_IN2_PORT        GPIOB
#define MOTOR_R_IN2_PIN         GPIO_PIN_15

/**
 * Motor direction helper macros.
 * Apply to BOTH left/right in sequence to set direction.
 */
#define MOTOR_L_FORWARD()  do {                                              \
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);     \
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);   \
} while(0)

#define MOTOR_L_REVERSE()  do {                                              \
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);   \
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);     \
} while(0)

#define MOTOR_L_BRAKE()    do {                                              \
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);     \
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);     \
} while(0)

#define MOTOR_L_COAST()    do {                                              \
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);   \
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);   \
} while(0)

#define MOTOR_R_FORWARD()  do {                                              \
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);     \
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);   \
} while(0)

#define MOTOR_R_REVERSE()  do {                                              \
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);   \
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);     \
} while(0)

#define MOTOR_R_BRAKE()    do {                                              \
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);     \
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);     \
} while(0)

#define MOTOR_R_COAST()    do {                                              \
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);   \
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);   \
} while(0)

/* ===================================================================== *
 *  SECTION 3  —  MOTOR STANDBY  (TB6612FNG STBY, active HIGH)
 *
 *  HIGH = driver enabled (normal operation)
 *  LOW  = standby        (outputs Hi-Z, power-save)
 *
 *  CubeMX: PB7 → GPIO Output, No pull, Push-pull, Low speed
 * ===================================================================== */

#define MOTOR_STBY_PORT         GPIOB
#define MOTOR_STBY_PIN          GPIO_PIN_7

#define MOTOR_ENABLE()   HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET)
#define MOTOR_DISABLE()  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET)
#define MOTOR_STBY_READ() (HAL_GPIO_ReadPin(MOTOR_STBY_PORT, MOTOR_STBY_PIN) == GPIO_PIN_SET)

/* ===================================================================== *
 *  SECTION 4  —  ENCODERS
 *
 *  LEFT encoder  → TIM2 (32-bit, PA0/PA1)
 *    CubeMX: TIM2 → Combined Channels: Encoder Mode
 *            Encoder Mode: TI1 and TI2
 *            Counter Period: 4294967295 (0xFFFF FFFF)
 *            Auto-Reload Preload: ENABLE
 *            CH1 Input: Direct (TI1),  PA0, AF01
 *            CH2 Input: Direct (TI2),  PA1, AF01
 *
 *  RIGHT encoder → TIM3 (16-bit, PB4/PB5)
 *    CubeMX: TIM3 → Combined Channels: Encoder Mode
 *            Encoder Mode: TI1 and TI2
 *            Counter Period: 65535 (0xFFFF)
 *            NVIC: TIM3 global interrupt ENABLED  ← for overflow tracking
 *            CH1 Input: Direct (TI1),  PB4, AF02
 *            CH2 Input: Direct (TI2),  PB5, AF02
 *
 *  NOTE: TIM3 is 16-bit. Track overflows in TIM3 IRQ to get 32-bit range.
 * ===================================================================== */

/** Left encoder A — PA0 — TIM2_CH1 */
#define ENC_L_A_PORT            GPIOA
#define ENC_L_A_PIN             GPIO_PIN_0
#define ENC_L_A_AF              GPIO_AF1_TIM2

/** Left encoder B — PA1 — TIM2_CH2 */
#define ENC_L_B_PORT            GPIOA
#define ENC_L_B_PIN             GPIO_PIN_1
#define ENC_L_B_AF              GPIO_AF1_TIM2

/** Left encoder timer */
#define ENC_L_TIM               TIM2
#define ENC_L_TIM_INSTANCE      TIM2        /**< for htim2 init          */
#define ENC_L_CNT               (TIM2->CNT) /**< direct 32-bit read      */

/** Right encoder A — PB4 — TIM3_CH1 */
#define ENC_R_A_PORT            GPIOB
#define ENC_R_A_PIN             GPIO_PIN_4
#define ENC_R_A_AF              GPIO_AF2_TIM3

/** Right encoder B — PB5 — TIM3_CH2 */
#define ENC_R_B_PORT            GPIOB
#define ENC_R_B_PIN             GPIO_PIN_5
#define ENC_R_B_AF              GPIO_AF2_TIM3

/** Right encoder timer (16-bit — extend with overflow counter in ISR) */
#define ENC_R_TIM               TIM3
#define ENC_R_TIM_INSTANCE      TIM3
#define ENC_R_CNT               ((uint16_t)(TIM3->CNT))
#define ENC_R_MAX_COUNT         65535U

/** Reset both encoder counters to 0 */
#define ENC_RESET_ALL()  do { TIM2->CNT = 0U; TIM3->CNT = 0U; } while(0)
#define ENC_RESET_LEFT() do { TIM2->CNT = 0U; } while(0)
#define ENC_RESET_RIGHT()do { TIM3->CNT = 0U; } while(0)

/* ===================================================================== *
 *  SECTION 5  —  CONTROL LOOP TIMER  (TIM5, no physical pin)
 *
 *  CubeMX: TIM5 → Timer Interrupt
 *          Prescaler  = 99        → 100 MHz / 100 = 1 MHz tick
 *          Period     = 999       → 1 MHz  / 1000 = 1 kHz interrupt
 *          Auto-Reload Preload: ENABLE
 *          NVIC: TIM5 global interrupt ENABLED, priority = 1
 * ===================================================================== */

#define CTRL_TIM_INSTANCE       TIM5
#define CTRL_TIM_PRESCALER      99U     /**< 100 MHz / (99+1) = 1 MHz   */
#define CTRL_TIM_PERIOD         999U    /**< 1 MHz  / (999+1) = 1 kHz   */
#define CTRL_TIM_PRIORITY       1U      /**< NVIC preemption priority    */

/* ===================================================================== *
 *  SECTION 6  —  IR EMITTERS  (6 independent AO3400A MOSFET gates)
 *
 *  Each emitter group is controlled by a separate GPIO pin so that
 *  individual sensor pairs can be fired independently, eliminating
 *  cross-talk between adjacent pairs.
 *
 *  All 6 pins: GPIO Output, No pull, Push-pull, Low speed
 *  Active HIGH (MOSFET gate pulled HIGH → MOSFET conducts → LED lights)
 *
 *  CubeMX: Configure as GPIO_Output for each pin below.
 *          NOTE: PA15 defaults to JTDI — must disable JTAG in CubeMX
 *                (System Core → SYS → Debug → Serial Wire only).
 *                PA15 then becomes free GPIO.
 *          NOTE: PB3 defaults to JTDO — same fix as PA15.
 * ===================================================================== */

/** Right Angle IR emitter — PA10 */
#define IR_EMIT_R_ANGLE_PORT    GPIOA
#define IR_EMIT_R_ANGLE_PIN     GPIO_PIN_10
#define IR_EMIT_R_ANGLE_ON()    HAL_GPIO_WritePin(IR_EMIT_R_ANGLE_PORT, IR_EMIT_R_ANGLE_PIN, GPIO_PIN_SET)
#define IR_EMIT_R_ANGLE_OFF()   HAL_GPIO_WritePin(IR_EMIT_R_ANGLE_PORT, IR_EMIT_R_ANGLE_PIN, GPIO_PIN_RESET)

/** Left Angle IR emitter — PA11 */
#define IR_EMIT_L_ANGLE_PORT    GPIOA
#define IR_EMIT_L_ANGLE_PIN     GPIO_PIN_11
#define IR_EMIT_L_ANGLE_ON()    HAL_GPIO_WritePin(IR_EMIT_L_ANGLE_PORT, IR_EMIT_L_ANGLE_PIN, GPIO_PIN_SET)
#define IR_EMIT_L_ANGLE_OFF()   HAL_GPIO_WritePin(IR_EMIT_L_ANGLE_PORT, IR_EMIT_L_ANGLE_PIN, GPIO_PIN_RESET)

/** Right Front IR emitter — PA15  ⚠ disable JTAG in CubeMX (SWD only) */
#define IR_EMIT_RF_PORT         GPIOA
#define IR_EMIT_RF_PIN          GPIO_PIN_15
#define IR_EMIT_RF_ON()         HAL_GPIO_WritePin(IR_EMIT_RF_PORT, IR_EMIT_RF_PIN, GPIO_PIN_SET)
#define IR_EMIT_RF_OFF()        HAL_GPIO_WritePin(IR_EMIT_RF_PORT, IR_EMIT_RF_PIN, GPIO_PIN_RESET)

/** Left Front IR emitter — PB3   ⚠ disable JTAG in CubeMX (SWD only) */
#define IR_EMIT_LF_PORT         GPIOB
#define IR_EMIT_LF_PIN          GPIO_PIN_3
#define IR_EMIT_LF_ON()         HAL_GPIO_WritePin(IR_EMIT_LF_PORT, IR_EMIT_LF_PIN, GPIO_PIN_SET)
#define IR_EMIT_LF_OFF()        HAL_GPIO_WritePin(IR_EMIT_LF_PORT, IR_EMIT_LF_PIN, GPIO_PIN_RESET)

/** Right Side IR emitter — PB2 */
#define IR_EMIT_R_SIDE_PORT     GPIOB
#define IR_EMIT_R_SIDE_PIN      GPIO_PIN_2
#define IR_EMIT_R_SIDE_ON()     HAL_GPIO_WritePin(IR_EMIT_R_SIDE_PORT, IR_EMIT_R_SIDE_PIN, GPIO_PIN_SET)
#define IR_EMIT_R_SIDE_OFF()    HAL_GPIO_WritePin(IR_EMIT_R_SIDE_PORT, IR_EMIT_R_SIDE_PIN, GPIO_PIN_RESET)

/** Left Side IR emitter — PB1 */
#define IR_EMIT_L_SIDE_PORT     GPIOB
#define IR_EMIT_L_SIDE_PIN      GPIO_PIN_1
#define IR_EMIT_L_SIDE_ON()     HAL_GPIO_WritePin(IR_EMIT_L_SIDE_PORT, IR_EMIT_L_SIDE_PIN, GPIO_PIN_SET)
#define IR_EMIT_L_SIDE_OFF()    HAL_GPIO_WritePin(IR_EMIT_L_SIDE_PORT, IR_EMIT_L_SIDE_PIN, GPIO_PIN_RESET)

/**
 * Fire all 6 emitters simultaneously (use for pulsed differential reads
 * when cross-talk is not a concern, e.g. open-space ambient calibration).
 */
#define IR_ALL_EMITTERS_ON()   do {   \
    IR_EMIT_R_ANGLE_ON();             \
    IR_EMIT_L_ANGLE_ON();             \
    IR_EMIT_RF_ON();                  \
    IR_EMIT_LF_ON();                  \
    IR_EMIT_R_SIDE_ON();              \
    IR_EMIT_L_SIDE_ON();              \
} while(0)

#define IR_ALL_EMITTERS_OFF()  do {   \
    IR_EMIT_R_ANGLE_OFF();            \
    IR_EMIT_L_ANGLE_OFF();            \
    IR_EMIT_RF_OFF();                 \
    IR_EMIT_LF_OFF();                 \
    IR_EMIT_R_SIDE_OFF();             \
    IR_EMIT_L_SIDE_OFF();             \
} while(0)

/* ===================================================================== *
 *  SECTION 7  —  IR RECEIVERS  (ADC1, 6-channel DMA circular scan)
 *
 *  Each TEFT4300 phototransistor: collector to 3.3 V via 10 kΩ pull-up,
 *  emitter to GND.  ADC pin reads the collector voltage.
 *  High ADC value = less light; low ADC value = more IR light = wall near.
 *  (Invert in software: differential = ambient_raw - lit_raw)
 *
 *  CubeMX:  ADC1 → Continuous conversion: OFF (we trigger manually)
 *           Scan conversion mode: ON
 *           DMA request: ENABLE
 *           DMA: ADC1, Circular, Half-word, Memory increment ON
 *           External trigger: Software start
 *           Sample time: 84 cycles per channel (good for 10kΩ source)
 *
 *  RANK ORDER matches the DMA buffer index in sensors.c:
 *    buf[IR_IDX_RS]     rank 1
 *    buf[IR_IDX_LS]     rank 2
 *    buf[IR_IDX_RF]     rank 3
 *    buf[IR_IDX_LF]     rank 4
 *    buf[IR_IDX_R_ANG]  rank 5
 *    buf[IR_IDX_L_ANG]  rank 6
 *    buf[IR_IDX_BATT]   rank 7  ← battery on same scan
 * ===================================================================== */

/* DMA buffer indices (must match CubeMX rank order) */
#define IR_IDX_RS          0U   /**< Right Side receiver                 */
#define IR_IDX_LS          1U   /**< Left  Side receiver                 */
#define IR_IDX_RF          2U   /**< Right Front receiver                */
#define IR_IDX_LF          3U   /**< Left  Front receiver                */
#define IR_IDX_R_ANG       4U   /**< Right Angle receiver                */
#define IR_IDX_L_ANG       5U   /**< Left  Angle receiver                */
#define IR_IDX_BATT        6U   /**< Battery voltage (PB0)              */
#define IR_ADC_BUF_LEN     7U   /**< Total DMA scan length               */

/* Physical pin definitions (for CubeMX GPIO/Analog setup) */

/** Right Side receiver — PA2 — ADC1_IN2, rank 1 */
#define IR_RECV_RS_PORT         GPIOA
#define IR_RECV_RS_PIN          GPIO_PIN_2
#define IR_RECV_RS_ADC_CH       ADC_CHANNEL_2
#define IR_RECV_RS_ADC_RANK     1U

/** Left Side receiver — PA3 — ADC1_IN3, rank 2 */
#define IR_RECV_LS_PORT         GPIOA
#define IR_RECV_LS_PIN          GPIO_PIN_3
#define IR_RECV_LS_ADC_CH       ADC_CHANNEL_3
#define IR_RECV_LS_ADC_RANK     2U

/** Right Front receiver — PA4 — ADC1_IN4, rank 3 */
#define IR_RECV_RF_PORT         GPIOA
#define IR_RECV_RF_PIN          GPIO_PIN_4
#define IR_RECV_RF_ADC_CH       ADC_CHANNEL_4
#define IR_RECV_RF_ADC_RANK     3U

/** Left Front receiver — PA5 — ADC1_IN5, rank 4 */
#define IR_RECV_LF_PORT         GPIOA
#define IR_RECV_LF_PIN          GPIO_PIN_5
#define IR_RECV_LF_ADC_CH       ADC_CHANNEL_5
#define IR_RECV_LF_ADC_RANK     4U

/** Right Angle receiver — PA6 — ADC1_IN6, rank 5 */
#define IR_RECV_R_ANG_PORT      GPIOA
#define IR_RECV_R_ANG_PIN       GPIO_PIN_6
#define IR_RECV_R_ANG_ADC_CH    ADC_CHANNEL_6
#define IR_RECV_R_ANG_ADC_RANK  5U

/** Left Angle receiver — PA7 — ADC1_IN7, rank 6 */
#define IR_RECV_L_ANG_PORT      GPIOA
#define IR_RECV_L_ANG_PIN       GPIO_PIN_7
#define IR_RECV_L_ANG_ADC_CH    ADC_CHANNEL_7
#define IR_RECV_L_ANG_ADC_RANK  6U

/** ADC peripheral */
#define IR_ADC_INSTANCE         ADC1

/* ===================================================================== *
 *  SECTION 8  —  BATTERY VOLTAGE ADC  (PB0, ADC1_IN8, rank 7)
 *
 *  Voltage divider: Vbat ──┬── 10 kΩ ──┬── 10 kΩ ── GND
 *                          └── PB0 (ADC)┘
 *  ADC reads Vbat / 2.  Ref = 3.3 V, 12-bit (4095 counts).
 *  Vbat = ADC_raw × (3.3 / 4095) × 2
 *
 *  CubeMX: PB0 → Analog (no pull).  Add as ADC1 channel IN8, rank 7.
 *  ⚠  PB0 must NOT be configured as GPIO_Output elsewhere.
 * ===================================================================== */

#define BATT_ADC_PORT           GPIOB
#define BATT_ADC_PIN            GPIO_PIN_0
#define BATT_ADC_CH             ADC_CHANNEL_8
#define BATT_ADC_RANK           7U
#define BATT_ADC_INSTANCE       ADC1

/** Convert raw 12-bit ADC value to battery voltage in volts (float) */
#define BATT_RAW_TO_VOLTS(raw)  ((float)(raw) * (3.3f / 4095.0f) * 2.0f)

/* ===================================================================== *
 *  SECTION 9  —  IMU / GYROSCOPE  (MPU6500, I2C1)
 *
 *  CubeMX:  I2C1 → I2C
 *           Speed mode:  Fast mode (400 kHz)
 *           PB8 → I2C1_SCL,  alternate function AF04
 *           PB9 → I2C1_SDA,  alternate function AF04
 *           Both pins: Open-drain, Pull-up (external 4.7 kΩ to 3.3 V)
 *
 *  ⚠  PB4 (ENC_R_A) and PB5 (ENC_R_B) are TIM3 — NOT I2C.
 *     PB8 and PB9 are the correct I2C1 pins on STM32F411 AF04.
 * ===================================================================== */

#define IMU_SCL_PORT            GPIOB
#define IMU_SCL_PIN             GPIO_PIN_8
#define IMU_SCL_AF              GPIO_AF4_I2C1

#define IMU_SDA_PORT            GPIOB
#define IMU_SDA_PIN             GPIO_PIN_9
#define IMU_SDA_AF              GPIO_AF4_I2C1

#define IMU_I2C_INSTANCE        I2C1
#define IMU_I2C_SPEED           400000U        /**< 400 kHz fast mode    */

/** MPU6500 7-bit I2C address (AD0 pin tied to GND → 0x68)              */
#define MPU6500_I2C_ADDR        0x68U
/** Shifted left 1 for HAL_I2C_xxx functions                            */
#define MPU6500_I2C_ADDR_8BIT   (MPU6500_I2C_ADDR << 1U)

/**
 * MPU6500 INT — PC13 — EXTI input, plain GPIO (no timer/analog AF exists
 * on PC13). Wired for future use; imu.c currently polls the IMU from the
 * 1 kHz TIM5 ISR and does not yet attach an EXTI callback to this pin.
 * PC13 output-drive limits (2 MHz / 3 mA, no current-source use) do not
 * apply here since this is an input.
 *
 * CubeMX: PC13 → GPIO_EXTI13, no pull (MPU6500 INT is push-pull by
 * default register config). Configure NVIC EXTI15_10 if/when imu.c
 * moves to interrupt-driven data-ready reads.
 */
#define IMU_INT_PORT             GPIOC
#define IMU_INT_PIN              GPIO_PIN_13

#define IMU_INT_READ()           (HAL_GPIO_ReadPin(IMU_INT_PORT, IMU_INT_PIN) == GPIO_PIN_SET)

/* ===================================================================== *
 *  SECTION 10  —  DIP SWITCHES  (4-bit mode selector, active LOW)
 *
 *  All pins: GPIO Input, Pull-up, no speed setting needed.
 *  Switch CLOSED (ON position) → pin pulled to GND → reads 0 → bit = 1.
 *
 *  CubeMX: Configure each as GPIO_Input with Pull-up.
 *  PC13 is no longer shared with any DIP switch — it's dedicated to
 *  MPU6500_INT now (see Section 9). No onboard LED is used either.
 * ===================================================================== */

/** DIP switch 1 — PA12 */
#define DIP1_PORT               GPIOA
#define DIP1_PIN                GPIO_PIN_12

/** DIP switch 2 — PB6 */
#define DIP2_PORT               GPIOB
#define DIP2_PIN                GPIO_PIN_6

/** DIP switch 3 — PB10 */
#define DIP3_PORT               GPIOB
#define DIP3_PIN                GPIO_PIN_10

/** DIP switch 4 — PC14 */
#define DIP4_PORT               GPIOC
#define DIP4_PIN                GPIO_PIN_14

/**
 * @brief  Read all 4 DIP switches into a single 0-15 mode value.
 *
 * @details Bit0=DIP1, Bit1=DIP2, Bit2=DIP3, Bit3=DIP4 (matches the
 *          "SW3 SW2 SW1 SW0" logging convention in main.c). Each
 *          switch is active LOW per this section's pull-up wiring —
 *          CLOSED (ON) reads GPIO_PIN_RESET and contributes a 1 bit.
 */
#define READ_DIP_SWITCHES() \
    ((uint8_t)( \
        ((HAL_GPIO_ReadPin(DIP1_PORT, DIP1_PIN) == GPIO_PIN_RESET) ? 0x01U : 0x00U) | \
        ((HAL_GPIO_ReadPin(DIP2_PORT, DIP2_PIN) == GPIO_PIN_RESET) ? 0x02U : 0x00U) | \
        ((HAL_GPIO_ReadPin(DIP3_PORT, DIP3_PIN) == GPIO_PIN_RESET) ? 0x04U : 0x00U) | \
        ((HAL_GPIO_ReadPin(DIP4_PORT, DIP4_PIN) == GPIO_PIN_RESET) ? 0x08U : 0x00U) \
    ))

/**
 * @brief Read all 4 DIP switches as a 4-bit value (0x00–0x0F).
 *        Bit 0 = DIP1, bit 3 = DIP4.  Switch ON → bit HIGH.
 * @return uint8_t mode number 0–15
 */
static inline uint8_t PINS_READ_DIP(void)
{
    uint8_t mode = 0U;
    if (HAL_GPIO_ReadPin(DIP1_PORT, DIP1_PIN) == GPIO_PIN_RESET) mode |= (1U << 0U);
    if (HAL_GPIO_ReadPin(DIP2_PORT, DIP2_PIN) == GPIO_PIN_RESET) mode |= (1U << 1U);
    if (HAL_GPIO_ReadPin(DIP3_PORT, DIP3_PIN) == GPIO_PIN_RESET) mode |= (1U << 2U);
    if (HAL_GPIO_ReadPin(DIP4_PORT, DIP4_PIN) == GPIO_PIN_RESET) mode |= (1U << 3U);
    return mode;
}

/* ===================================================================== *
 *  SECTION 11  —  TEST / USER BUTTON  (active LOW)
 *
 *  CubeMX: PC15 → GPIO_Input, Pull-up
 *  Press = GND → reads GPIO_PIN_RESET → pressed.
 * ===================================================================== */

#define BTN_PORT                GPIOC
#define BTN_PIN                 GPIO_PIN_15

/** Returns non-zero (true) when button is pressed */
#define BTN_PRESSED()           (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET)

/**
 * @brief Blocking wait until button is pressed and released.
 *        Includes 20 ms software debounce.
 */
static inline void BTN_WAIT_PRESS(void)
{
    while (!BTN_PRESSED()) {}
    HAL_Delay(20U);   /* debounce */
    while (BTN_PRESSED()) {}
    HAL_Delay(20U);
}

/* ===================================================================== *
 *  SECTION 12  —  CUBEMX PERIPHERAL SUMMARY (reference only)
 *
 *  Copy these settings into STM32CubeMX exactly as shown.
 *
 *  ┌─────────────┬───────────────┬────────────────────────────────────┐
 *  │ Peripheral  │ Mode          │ Key settings                        │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ TIM1        │ PWM Gen CH1+2 │ PSC=0, ARR=4999, Preload=EN        │
 *  │             │               │ CH1=PA8 AF01, CH2=PA9 AF01         │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ TIM2        │ Encoder TI1+2 │ Period=0xFFFFFFFF, PA0/PA1 AF01    │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ TIM3        │ Encoder TI1+2 │ Period=0xFFFF, PB4/PB5 AF02        │
 *  │             │               │ NVIC: TIM3 global IRQ ENABLED       │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ TIM5        │ Timer Intr    │ PSC=99, ARR=999 → 1 kHz            │
 *  │             │               │ NVIC: TIM5 global IRQ, priority 1  │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ ADC1        │ Scan+DMA      │ 7 ranks (IN2–IN8), 84cyc each      │
 *  │             │               │ DMA: Circular, Half-word, MemInc   │
 *  │             │               │ Trigger: Software                   │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ I2C1        │ I2C 400 kHz   │ PB8 SCL AF04, PB9 SDA AF04        │
 *  │             │               │ External 4.7 kΩ pull-ups on both   │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ GPIO OUT    │ Push-pull     │ PB1,PB2,PB3,PB7,                   │
 *  │             │               │ PB12,PB13,PB14,PB15,               │
 *  │             │               │ PA10,PA11,PA15                     │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ GPIO IN PU  │ Pull-up       │ PA12,PB6,PB10,PC14,PC15            │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ GPIO IN     │ No pull, EXTI │ PC13 (MPU6500_INT)                 │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ GPIO Analog │ No pull       │ PA2,PA3,PA4,PA5,PA6,PA7,PB0        │
 *  ├─────────────┼───────────────┼────────────────────────────────────┤
 *  │ SYS/Debug   │ Serial Wire   │ Disable JTAG → frees PA15 + PB3    │
 *  └─────────────┴───────────────┴────────────────────────────────────┘
 * ===================================================================== */

/* ===================================================================== *
 *  SECTION 13  —  COMPLETE PIN CONFLICT MATRIX (verified)
 *
 *  Pin   Signal        Peripheral  Notes
 *  ────  ────────────  ──────────  ──────────────────────────────────
 *  PA0   L_EN_A        TIM2_CH1    Left encoder A                   ✓
 *  PA1   L_EN_B        TIM2_CH2    Left encoder B                   ✓
 *  PA2   RS_RE         ADC1_IN2    Right side IR receiver           ✓
 *  PA3   LS_RE         ADC1_IN3    Left  side IR receiver           ✓
 *  PA4   RF_RE         ADC1_IN4    Right front IR receiver          ✓
 *  PA5   LF_RE         ADC1_IN5    Left  front IR receiver          ✓
 *  PA6   R_AnRE        ADC1_IN6    Right angle IR receiver          ✓
 *  PA7   L_AnRE        ADC1_IN7    Left  angle IR receiver          ✓
 *  PA8   LM_SIG        TIM1_CH1    Left  motor PWM                  ✓
 *  PA9   RM_SIG        TIM1_CH2    Right motor PWM                  ✓
 *  PA10  R_ANGLE_EM    GPIO OUT    Right angle IR emitter           ✓
 *  PA11  L_ANGLE_EM    GPIO OUT    Left  angle IR emitter           ✓
 *  PA12  DIP1          GPIO IN PU  DIP switch 1                     ✓
 *  PA15  RF_EM         GPIO OUT    Right front IR emitter           ✓
 *        ⚠ PA15 = JTDI by default → disable JTAG in CubeMX
 *  PB0   VOLTMETER     ADC1_IN8    Battery voltage divider          ✓
 *  PB1   L_SIDE_EM     GPIO OUT    Left  side IR emitter            ✓
 *  PB2   R_SIDE_EM     GPIO OUT    Right side IR emitter            ✓
 *  PB3   LF_EM         GPIO OUT    Left front IR emitter            ✓
 *        ⚠ PB3 = JTDO by default → disable JTAG in CubeMX
 *  PB4   R_EN_A        TIM3_CH1    Right encoder A                  ✓
 *  PB5   R_EN_B        TIM3_CH2    Right encoder B                  ✓
 *  PB6   DIP2          GPIO IN PU  DIP switch 2                     ✓
 *  PB7   STBY          GPIO OUT    Motor driver standby             ✓
 *  PB8   G_SCL         I2C1_SCL    IMU I2C clock                    ✓
 *  PB9   G_SDA         I2C1_SDA    IMU I2C data                     ✓
 *  PB10  DIP3          GPIO IN PU  DIP switch 3                     ✓
 *  PB12  LM_CNT1       GPIO OUT    Left  motor AIN1 (dir)           ✓
 *  PB13  LM_CNT2       GPIO OUT    Left  motor AIN2 (dir)           ✓
 *  PB14  RM_CNT1       GPIO OUT    Right motor BIN1 (dir)           ✓
 *  PB15  RM_CNT2       GPIO OUT    Right motor BIN2 (dir)           ✓
 *  PC13  MPU6500_INT   GPIO IN     IMU interrupt (no LED, no DIP)   ✓
 *  PC14  DIP4          GPIO IN PU  DIP switch 4                     ✓
 *  PC15  TB1           GPIO IN PU  Test/user button                 ✓
 *
 *  Onboard LED and buzzer are NOT wired — no free pins remain on the
 *  UFQFPN48 package (PB11 and PC0–PC5 don't exist on this package).
 *
 *  RESERVED (debug — keep off-limits):
 *  PA13  SWDIO         SWD         Debug (keep reserved)
 *  PA14  SWDCLK        SWD         Debug (keep reserved)
 * ===================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* PINS_H */
