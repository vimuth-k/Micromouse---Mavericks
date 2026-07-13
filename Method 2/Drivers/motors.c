/**
 * @file    motors.c
 * @brief   TB6612FNG dual H-bridge motor driver — implementation.
 *
 * @details This module is the only file that writes to:
 *            - TIM1->CCR1 and TIM1->CCR2  (PWM duty cycle)
 *            - PB12 / PB13  (left  motor AIN1 / AIN2)
 *            - PB14 / PB15  (right motor BIN1 / BIN2)
 *            - PB10          (STBY — driver enable)
 *
 *          TB6612FNG TRUTH TABLE (per channel)
 *          ────────────────────────────────────
 *          xIN1  xIN2  PWM    Output      Mode
 *          ────  ────  ───    ──────────  ────────────────
 *          H     L     duty   Forward     Normal drive
 *          L     H     duty   Reverse     Normal drive
 *          H     H     any    Brake GND   Short-circuit brake
 *          L     L     any    Coast       Free-wheel (Hi-Z)
 *          any   any   any    Hi-Z        STBY = LOW (standby)
 *
 *          SIGNED SPEED CONVENTION
 *          ────────────────────────
 *          Positive speed → motor shaft rotates such that the robot
 *          moves FORWARD (after polarity correction from config.h).
 *          Negative speed → robot moves BACKWARD.
 *          The mapping from "forward" to actual xIN1/xIN2 levels
 *          depends on LEFT_MOTOR_POL and RIGHT_MOTOR_POL in config.h.
 *
 *          POLARITY CORRECTION
 *          ────────────────────
 *          Motors on opposite sides of the chassis are physically
 *          mirrored.  Without polarity correction both motors would
 *          spin in opposite directions for the same PWM sign, causing
 *          the robot to spin in place.  Set LEFT_MOTOR_POL and
 *          RIGHT_MOTOR_POL in config.h to +1 or -1 during Mode 2
 *          (motor test) until both wheels roll forward together.
 *
 *          DIRECT REGISTER WRITES
 *          ───────────────────────
 *          The 1 kHz PID control loop calls motors_set() every
 *          millisecond.  Using HAL_TIM_PWM functions inside an ISR
 *          adds unnecessary overhead.  This module writes directly to
 *          TIM1->CCR1 and TIM1->CCR2 for minimal latency.  TIM1 is
 *          initialised by main.c before the ISR starts.
 *
 * @author  VDawn
 * @date    2026
 */

#include "motors.h"
#include "pins.h"
#include "config.h"
#include "main.h"       /* htim1 extern */
#include "utils.h"      /* CLAMP macro  */

/* =========================================================================
 * PRIVATE STATE
 * ======================================================================= */

/** Tracks current signed PWM values for diagnostics. */
static volatile int32_t s_pwm_left  = 0;
static volatile int32_t s_pwm_right = 0;

/** Tracks STBY state without reading the GPIO every time. */
static bool s_enabled = false;

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Apply a signed PWM value to the left motor hardware.
 *
 * @details Handles the three TB6612FNG operating modes (forward,
 *          reverse, coast) and applies LEFT_MOTOR_POL from config.h.
 *          Writes directly to TIM1->CCR1 for minimal ISR latency.
 *
 * @param  speed  Signed PWM count after polarity has been applied.
 *                Expected range: [-PWM_MAX, PWM_MAX].
 */
static void apply_left(int32_t speed)
{
    if (speed > 0)
    {
        MOTOR_L_FORWARD();
        MOTOR_L_CCR = (uint32_t)speed;
    }
    else if (speed < 0)
    {
        MOTOR_L_REVERSE();
        MOTOR_L_CCR = (uint32_t)(-speed);
    }
    else
    {
        MOTOR_L_COAST();
        MOTOR_L_CCR = 0U;
    }
}

/**
 * @brief  Apply a signed PWM value to the right motor hardware.
 *
 * @details Identical to apply_left() but targets TIM1->CCR2 and
 *          BIN1/BIN2 direction pins.
 *
 * @param  speed  Signed PWM count after polarity has been applied.
 */
static void apply_right(int32_t speed)
{
    if (speed > 0)
    {
        MOTOR_R_FORWARD();
        MOTOR_R_CCR = (uint32_t)speed;
    }
    else if (speed < 0)
    {
        MOTOR_R_REVERSE();
        MOTOR_R_CCR = (uint32_t)(-speed);
    }
    else
    {
        MOTOR_R_COAST();
        MOTOR_R_CCR = 0U;
    }
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the motor driver module.
 *
 * @details TIM1 PWM channels must already be started by main.c before
 *          this is called.  This function sets all direction and STBY
 *          pins to their safe default states and zeroes both CCRs.
 *
 * @return MM_OK always.
 */
MmResult_t motors_init(void)
{
    /* Safe defaults — driver inactive, all outputs low */
    MOTOR_L_COAST();
    MOTOR_R_COAST();

    MOTOR_L_CCR = 0U;
    MOTOR_R_CCR = 0U;

    MOTOR_DISABLE();
    s_enabled    = false;
    s_pwm_left   = 0;
    s_pwm_right  = 0;

    return MM_OK;
}

/**
 * @brief  Enable the motor driver (STBY → HIGH).
 */
void motors_enable(void)
{
    MOTOR_ENABLE();
    s_enabled = true;
}

/**
 * @brief  Disable the motor driver (STBY → LOW).
 *
 * @details Zeroes outputs before pulling STBY low to prevent a
 *          momentary full-voltage spike on re-enable.
 */
void motors_disable(void)
{
    MOTOR_L_CCR = 0U;
    MOTOR_R_CCR = 0U;
    MOTOR_L_COAST();
    MOTOR_R_COAST();

    MOTOR_DISABLE();
    s_enabled   = false;
    s_pwm_left  = 0;
    s_pwm_right = 0;
}

/**
 * @brief  Return true when the driver is enabled.
 */
bool motors_is_enabled(void)
{
    return s_enabled;
}

/* =========================================================================
 * PUBLIC API — SPEED CONTROL
 * ======================================================================= */

/**
 * @brief  Set left motor output.
 *
 * @details Steps:
 *          1. Apply LEFT_MOTOR_POL: flips the sign if the motor is
 *             wired in reverse on this chassis.
 *          2. Clamp to [-PWM_MAX, PWM_MAX].
 *          3. Apply deadband: if |speed| < PWM_DEADBAND and speed ≠ 0,
 *             lift to ±PWM_DEADBAND so the motor actually moves.
 *          4. Write direction pins and CCR register via apply_left().
 *          5. Cache the value for diagnostics.
 *
 * @param  speed  Signed PWM count. Positive = robot forward.
 */
void motor_left_set(int32_t speed)
{
    /* 1. Polarity correction */
    int32_t corrected = speed * LEFT_MOTOR_POL;

    /* 2. Clamp */
    corrected = CLAMP(corrected, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

    /* 3. Deadband — lift small non-zero requests above motor stall */
    if ((corrected > 0) && (corrected < (int32_t)PWM_DEADBAND))
    {
        corrected = (int32_t)PWM_DEADBAND;
    }
    else if ((corrected < 0) && (corrected > -(int32_t)PWM_DEADBAND))
    {
        corrected = -(int32_t)PWM_DEADBAND;
    }

    /* 4. Apply to hardware */
    apply_left(corrected);

    /* 5. Cache for diagnostics */
    s_pwm_left = corrected;
}

/**
 * @brief  Set right motor output.
 *
 * @details Identical flow to motor_left_set() with RIGHT_MOTOR_POL
 *          and TIM1->CCR2 / BIN1/BIN2.
 *
 * @param  speed  Signed PWM count. Positive = robot forward.
 */
void motor_right_set(int32_t speed)
{
    int32_t corrected = speed * RIGHT_MOTOR_POL;
    corrected = CLAMP(corrected, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

    if ((corrected > 0) && (corrected < (int32_t)PWM_DEADBAND))
    {
        corrected = (int32_t)PWM_DEADBAND;
    }
    else if ((corrected < 0) && (corrected > -(int32_t)PWM_DEADBAND))
    {
        corrected = -(int32_t)PWM_DEADBAND;
    }

    apply_right(corrected);
    s_pwm_right = corrected;
}

/**
 * @brief  Set both motors in a single call.
 *
 * @details Preferred API for the 1 kHz PID tick because it avoids
 *          two separate function call overheads.  Applies polarity
 *          and deadband to each wheel independently.
 *
 * @param  left   Left  motor signed PWM. Range: [-PWM_MAX, PWM_MAX].
 * @param  right  Right motor signed PWM. Range: [-PWM_MAX, PWM_MAX].
 */
void motors_set(int32_t left, int32_t right)
{
    /* Apply polarity */
    int32_t cl = left  * LEFT_MOTOR_POL;
    int32_t cr = right * RIGHT_MOTOR_POL;

    /* Clamp */
    cl = CLAMP(cl, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);
    cr = CLAMP(cr, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

    /* Deadband */
    if ((cl > 0) && (cl < (int32_t)PWM_DEADBAND)) { cl =  (int32_t)PWM_DEADBAND; }
    if ((cl < 0) && (cl > -(int32_t)PWM_DEADBAND)){ cl = -(int32_t)PWM_DEADBAND; }
    if ((cr > 0) && (cr < (int32_t)PWM_DEADBAND)) { cr =  (int32_t)PWM_DEADBAND; }
    if ((cr < 0) && (cr > -(int32_t)PWM_DEADBAND)){ cr = -(int32_t)PWM_DEADBAND; }

    /* Apply both — direction first, then CCR for glitch-free update */
    apply_left(cl);
    apply_right(cr);

    s_pwm_left  = cl;
    s_pwm_right = cr;
}

/* =========================================================================
 * PUBLIC API — STOP MODES
 * ======================================================================= */

/**
 * @brief  Coast stop — motors free-wheel, driver stays enabled.
 *
 * @details Sets direction pins low and zeroes PWM.
 *          Does NOT pull STBY low — driver remains active so the next
 *          motors_set() call takes effect immediately.
 */
void motors_coast(void)
{
    MOTOR_L_COAST();
    MOTOR_R_COAST();
    MOTOR_L_CCR = 0U;
    MOTOR_R_CCR = 0U;
    s_pwm_left  = 0;
    s_pwm_right = 0;
}

/**
 * @brief  Active brake — short-circuit stop, driver stays enabled.
 *
 * @details Sets both direction pins HIGH on each channel.  The TB6612
 *          shorts the motor terminals together through the low-side
 *          FETs, creating regenerative braking.  This stops the robot
 *          significantly faster than coasting.
 *
 *          ⚠ Sustained short-circuit draws current even at standstill.
 *          Always follow with motors_coast() after TURN_SETTLE_MS ms.
 */
void motors_brake(void)
{
    MOTOR_L_BRAKE();
    MOTOR_R_BRAKE();
    MOTOR_L_CCR = (uint32_t)PWM_MAX;
    MOTOR_R_CCR = (uint32_t)PWM_MAX;
    s_pwm_left  = 0;
    s_pwm_right = 0;
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a MotorState_t snapshot of current driver state.
 *
 * @param[out] state  Pointer to the struct to fill. Must not be NULL.
 */
void motors_get_state(MotorState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->pwm_left  = s_pwm_left;
    state->pwm_right = s_pwm_right;
    state->enabled   = s_enabled;
}
