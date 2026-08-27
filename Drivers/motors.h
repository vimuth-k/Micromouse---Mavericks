/**
 * @file    motors.h
 * @brief   TB6612FNG dual H-bridge motor driver — public API.
 *
 * @details Provides a signed-speed interface over the TB6612FNG.
 *          Callers pass a speed value in the range [-PWM_MAX, +PWM_MAX].
 *          Positive = forward, negative = reverse, zero = coast.
 *
 *          This module owns:
 *            - TIM1 CCR1/CCR2 registers (PWM duty cycle)
 *            - PB12/PB13 direction pins (left motor  AIN1/AIN2)
 *            - PB14/PB15 direction pins (right motor BIN1/BIN2)
 *            - PB10 standby pin         (STBY, active HIGH)
 *
 *          No other module writes to these registers or pins.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          pins.h   — pin macros and direction helper macros
 *          config.h — PWM_MAX, PWM_MIN, LEFT/RIGHT_MOTOR_POL, PWM_DEADBAND
 *          main.h   — htim1 handle (extern)
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef MOTORS_H
#define MOTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief Motor identifier.
 */
typedef enum
{
    MOTOR_LEFT  = 0U,
    MOTOR_RIGHT = 1U,
} Motor_t;

/**
 * @brief Motor state snapshot — populated by motors_get_state().
 */
typedef struct
{
    int32_t  pwm_left;       /**< Current left  PWM value [-PWM_MAX, PWM_MAX] */
    int32_t  pwm_right;      /**< Current right PWM value [-PWM_MAX, PWM_MAX] */
    bool     enabled;        /**< true when STBY pin is HIGH (driver active)   */
} MotorState_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the motor driver module.
 *
 * @details - Starts TIM1 PWM on CH1 (PA8) and CH2 (PA9) at 0 % duty.
 *          - Sets all direction pins (PB12–PB15) LOW.
 *          - Pulls STBY LOW (driver in standby — motors cannot move).
 *          Call once during system bring-up, before motors_enable().
 *
 * @return MM_OK always (TIM1 is assumed started by main.c CubeMX init).
 */
MmResult_t motors_init(void);

/**
 * @brief  Enable the motor driver (STBY → HIGH).
 *
 * @details Pulls PB10 (STBY) HIGH, allowing the TB6612FNG to drive
 *          current through both motor channels.
 *          Call this before issuing any move commands.
 */
void motors_enable(void);

/**
 * @brief  Disable the motor driver (STBY → LOW).
 *
 * @details Pulls PB10 (STBY) LOW.  Both motor outputs go Hi-Z.
 *          The motors coast to a stop — this is NOT an active brake.
 *          Also zeroes both CCR registers and resets direction pins.
 *          Call at the end of every run, in safety.c on fault detection,
 *          and in HardFault_Handler.
 */
void motors_disable(void);

/**
 * @brief  Return true when the driver is enabled (STBY HIGH).
 *
 * @return true  Motor driver is active.
 * @return false Motor driver is in standby.
 */
bool motors_is_enabled(void);

/* =========================================================================
 * SPEED CONTROL
 * ======================================================================= */

/**
 * @brief  Set left motor output.
 *
 * @details Maps the signed speed value to TB6612FNG direction pins and
 *          TIM1 CCR1 PWM duty cycle:
 *            speed > 0  : AIN1=H AIN2=L, CCR1 = speed
 *            speed < 0  : AIN1=L AIN2=H, CCR1 = |speed|
 *            speed == 0 : AIN1=L AIN2=L, CCR1 = 0  (coast)
 *          Physical direction polarity is applied from LEFT_MOTOR_POL.
 *          Values outside [-PWM_MAX, PWM_MAX] are clamped silently.
 *
 * @param  speed  Signed PWM count. Positive = forward, negative = reverse.
 *                Range: [-PWM_MAX, PWM_MAX] = [-4999, 4999].
 */
void motor_left_set(int32_t speed);

/**
 * @brief  Set right motor output.
 *
 * @details Identical behaviour to motor_left_set() but for:
 *            BIN1/BIN2 direction pins (PB14/PB15)
 *            TIM1 CCR2 PWM register
 *          Physical direction polarity applied from RIGHT_MOTOR_POL.
 *
 * @param  speed  Signed PWM count. Range: [-PWM_MAX, PWM_MAX].
 */
void motor_right_set(int32_t speed);

/**
 * @brief  Set both motors in a single call.
 *
 * @details Equivalent to calling motor_left_set(left) then
 *          motor_right_set(right), but marginally faster because both
 *          CCR registers are updated in sequence without a function
 *          call boundary.  Use this in the 1 kHz PID tick.
 *
 * @param  left   Left  motor signed PWM. Range: [-PWM_MAX, PWM_MAX].
 * @param  right  Right motor signed PWM. Range: [-PWM_MAX, PWM_MAX].
 */
void motors_set(int32_t left, int32_t right);

/* =========================================================================
 * STOP MODES
 * ======================================================================= */

/**
 * @brief  Coast stop — both motors output zero, driver remains enabled.
 *
 * @details Sets AIN1=AIN2=L, BIN1=BIN2=L, CCR1=CCR2=0.
 *          Motors spin down under their own inertia (free-wheel).
 *          STBY remains HIGH — the driver stays active.
 *          Use between moves when the next command is imminent.
 */
void motors_coast(void);

/**
 * @brief  Active brake — short-circuit stop, driver remains enabled.
 *
 * @details Sets AIN1=AIN2=H, BIN1=BIN2=H, CCR1=CCR2=PWM_MAX.
 *          The H-bridge shorts each motor terminal to GND, generating
 *          a strong back-EMF braking torque.
 *          Use at cell boundaries to stop the robot precisely.
 *          Follow with motors_coast() after TURN_SETTLE_MS to prevent
 *          motor heating from sustained short-circuit current.
 */
void motors_brake(void);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a MotorState_t snapshot of current driver state.
 *
 * @details Reads the live CCR register values and the STBY pin state.
 *          Thread-safe: called from main loop, not from the 1 kHz ISR.
 *
 * @param[out] state  Pointer to the MotorState_t struct to fill.
 *                    Must not be NULL.
 */
void motors_get_state(MotorState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
