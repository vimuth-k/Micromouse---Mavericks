/**
 * @file    pid.h
 * @brief   Generic discrete-time PID controller — public API.
 *
 * @details A single reusable PID implementation used by every control
 *          loop in the micromouse firmware.  The same code serves four
 *          completely different physical systems:
 *
 *          INSTANCES IN USE
 *          ─────────────────
 *          pid_speed_left   : Left  wheel speed → PWM output
 *          pid_speed_right  : Right wheel speed → PWM output
 *          pid_straight     : Encoder count difference → correction
 *          pid_heading      : Gyro yaw angle → correction
 *          pid_turn         : Gyro heading error during pivot turns
 *
 *          All five instances share this struct and these functions.
 *          Gains and limits are configured per-instance at init time
 *          from constants in config.h.
 *
 *          ALGORITHM
 *          ──────────
 *          Standard position-form discrete PID evaluated at fixed
 *          intervals (CTRL_LOOP_HZ = 1 kHz, Δt = 1 ms):
 *
 *            error[n]  = setpoint − measurement
 *
 *            P term    = Kp × error[n]
 *
 *            I term    = Ki × Σ error[k]   (accumulated integral)
 *
 *            D term    = Kd × (error[n] − error[n−1])
 *                        (derivative on error, not measurement,
 *                         because setpoint changes cause a D spike
 *                         with measurement-based derivative too)
 *
 *            output[n] = P + I + D
 *
 *          ANTI-WINDUP
 *          ────────────
 *          When the output saturates (hits output_limit), the integral
 *          keeps growing — "winding up" — even though increasing it
 *          further has no effect.  When the error reverses and the
 *          output should desaturate, the wound-up integral delays the
 *          response by seconds.  This module uses symmetric clamping:
 *          the integral is hard-clamped to [-i_limit, +i_limit] every
 *          tick.  Choose i_limit conservatively — for the speed PID,
 *          SPEED_INTEG_LIMIT = 2000 means the I term can contribute
 *          at most Ki × 2000 = 9.0 × 2000 = 18000 to the output,
 *          far above PWM_MAX (4999), so it is effectively the integral
 *          that is limited, not the output.  Tune i_limit tighter to
 *          reduce windup at the cost of slower steady-state correction.
 *
 *          DERIVATIVE FILTER
 *          ──────────────────
 *          Raw derivative is noisy — encoder quantisation (1–2 counts
 *          per tick) appears as large D spikes.  An optional N-tap
 *          moving average filter smooths the derivative term without
 *          introducing significant phase lag.  Enabled by setting
 *          d_filter_n > 1 in pid_init_ex().  Default: no filter (n=1).
 *
 *          CALL CONVENTION
 *          ────────────────
 *          Call pid_update() exactly once per control loop tick with
 *          the signed error (setpoint − measurement).  Do not pass
 *          the measurement and setpoint separately — the caller computes
 *          the error, giving full control over sign convention.
 *
 *          pid_reset() must be called before every new move to clear
 *          the integral and previous-error state.  Stale integral from
 *          the previous run will cause an immediate output spike on the
 *          first tick of the new move.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h — CTRL_LOOP_DT (for documentation; not used in
 *                     the implementation since gains are already
 *                     per-tick, not per-second)
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * CONSTANTS
 * ======================================================================= */

/** Maximum number of taps in the derivative moving-average filter. */
#define PID_D_FILTER_MAX_TAPS   8U

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  PID controller instance.
 *
 * @details One struct per control loop.  All fields are private — access
 *          only via the API functions below.  Declared in the header so
 *          callers can allocate instances on the stack or as statics
 *          without a heap allocation.
 *
 *          Field layout is intentional: gains first (read-only after
 *          init), then state (read-write per tick), then limits last.
 */
typedef struct
{
    /* ── Gains (set by pid_init, read by pid_update) ──────────────── */
    float kp;              /**< Proportional gain                        */
    float ki;              /**< Integral gain                            */
    float kd;              /**< Derivative gain                          */

    /* ── Per-tick state (modified by pid_update, cleared by pid_reset) */
    float integral;        /**< Accumulated error sum (anti-windup clamped) */
    float prev_error;      /**< Error from the previous tick (for D term) */

    /* ── Derivative filter state ───────────────────────────────────── */
    float   d_buf[PID_D_FILTER_MAX_TAPS]; /**< Circular buffer for D term */
    uint8_t d_buf_idx;     /**< Next write index into d_buf              */
    uint8_t d_filter_n;    /**< Number of taps (1 = no filter)           */
    float   d_buf_sum;     /**< Running sum of d_buf (avoids re-summing) */

    /* ── Limits ────────────────────────────────────────────────────── */
    float i_limit;         /**< Integral clamped to [-i_limit, +i_limit] */
    float out_limit;       /**< Output  clamped to [-out_limit, +out_limit] */

    /* ── Diagnostics ───────────────────────────────────────────────── */
    float last_p;          /**< Last P term (for tuning display)         */
    float last_i;          /**< Last I term (for tuning display)         */
    float last_d;          /**< Last D term (for tuning display)         */
    float last_output;     /**< Last total output before clamping        */
} PID_t;

/**
 * @brief  PID term snapshot — populated by pid_get_terms().
 *
 * @details Used by diagnostics.c to display individual
 *          PID contributions during tuning without exposing PID_t internals.
 */
typedef struct
{
    float p;        /**< Proportional term contribution    */
    float i;        /**< Integral     term contribution    */
    float d;        /**< Derivative   term contribution    */
    float output;   /**< Total output (sum of p + i + d)  */
    float error;    /**< Current error passed to last tick */
    float integral; /**< Current integral accumulator      */
} PidTerms_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise a PID instance with gains, limits, and no D filter.
 *
 * @details Sets gains and limits.  Zeroes all state (integral,
 *          prev_error, D filter buffer).  Equivalent to pid_init_ex()
 *          with d_filter_n = 1 (no derivative smoothing).
 *
 *          Typical call for the speed PID:
 *            pid_init(&pid_speed_l,
 *                     KP_SPEED, KI_SPEED, KD_SPEED,
 *                     SPEED_INTEG_LIMIT, (float)PWM_MAX);
 *
 * @param[out] pid        PID instance to initialise. Must not be NULL.
 * @param[in]  kp         Proportional gain.
 * @param[in]  ki         Integral gain.
 * @param[in]  kd         Derivative gain.
 * @param[in]  i_limit    Integral clamp magnitude (symmetric).
 * @param[in]  out_limit  Output  clamp magnitude (symmetric).
 */
void pid_init(PID_t *pid,
              float  kp, float ki, float kd,
              float  i_limit, float out_limit);

/**
 * @brief  Initialise a PID instance with D-term moving-average filter.
 *
 * @details Like pid_init() but also enables a d_filter_n-tap moving
 *          average on the derivative term to reduce quantisation noise.
 *          d_filter_n must be in [1, PID_D_FILTER_MAX_TAPS].
 *          d_filter_n = 1 disables filtering (same as pid_init()).
 *          d_filter_n = 4 averages the last 4 derivative samples —
 *          recommended for the speed PID when encoder resolution is low.
 *
 * @param[out] pid         PID instance to initialise. Must not be NULL.
 * @param[in]  kp          Proportional gain.
 * @param[in]  ki          Integral gain.
 * @param[in]  kd          Derivative gain.
 * @param[in]  i_limit     Integral clamp magnitude.
 * @param[in]  out_limit   Output  clamp magnitude.
 * @param[in]  d_filter_n  Number of taps in D moving average (1–8).
 */
void pid_init_ex(PID_t  *pid,
                 float   kp, float ki, float kd,
                 float   i_limit, float out_limit,
                 uint8_t d_filter_n);

/**
 * @brief  Zero all per-tick state without changing gains or limits.
 *
 * @details Resets: integral, prev_error, D filter buffer and sum,
 *          d_buf_idx, and all last_xxx diagnostic fields.
 *          Gains (kp, ki, kd), limits (i_limit, out_limit), and
 *          d_filter_n are NOT changed.
 *
 *          Call before every new move command so the new move starts
 *          with a clean controller state.  Stale integral from a
 *          previous straight run will cause an unwanted initial kick
 *          at the start of the next turn, and vice versa.
 *
 * @param[out] pid  PID instance to reset. Must not be NULL.
 */
void pid_reset(PID_t *pid);

/* =========================================================================
 * RUNTIME
 * ======================================================================= */

/**
 * @brief  Compute one PID output step.
 *
 * @details Call exactly once per control loop tick.
 *          The error sign convention is: error = setpoint − measurement.
 *          Positive error → output is positive → corrects toward setpoint.
 *
 *          Computation steps:
 *            1. P = Kp × error
 *            2. integral += error
 *               integral  = clamp(integral, -i_limit, +i_limit)
 *            3. I = Ki × integral
 *            4. d_raw  = error − prev_error
 *               d_filtered = moving_average(d_raw, d_filter_n)
 *            5. D = Kd × d_filtered
 *            6. output = clamp(P + I + D, -out_limit, +out_limit)
 *            7. prev_error = error
 *
 * @param[in,out] pid    PID instance. Must not be NULL.
 * @param[in]     error  Signed error for this tick (setpoint − measurement).
 *
 * @return float  Clamped output in [-out_limit, +out_limit].
 */
float pid_update(PID_t *pid, float error);

/* =========================================================================
 * GAIN ADJUSTMENT (runtime tuning)
 * ======================================================================= */

/**
 * @brief  Update all three gains at runtime without resetting state.
 *
 * @details Allows PID gains to be adjusted via UART or DIP switch modes
 *          during a tuning session without stopping the robot.
 *          The integral and derivative state are preserved — only the
 *          gain multipliers change.  If a large gain change causes
 *          instability, call pid_reset() after pid_set_gains().
 *
 * @param[out] pid  PID instance. Must not be NULL.
 * @param[in]  kp   New proportional gain.
 * @param[in]  ki   New integral gain.
 * @param[in]  kd   New derivative gain.
 */
void pid_set_gains(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief  Update the output clamp limit at runtime.
 *
 * @details Allows the speed PID maximum output to be reduced for
 *          search runs (to limit top speed) and restored for speed
 *          runs, without reinitialising the controller.
 *
 * @param[out] pid       PID instance. Must not be NULL.
 * @param[in]  out_limit New output limit magnitude (must be > 0).
 */
void pid_set_output_limit(PID_t *pid, float out_limit);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a PidTerms_t snapshot of the last computed step.
 *
 * @details Returns the cached last_p, last_i, last_d, last_output
 *          values from the most recent pid_update() call.
 *          Safe to call from the main loop between ticks.
 *          Do NOT call from inside the 1 kHz ISR alongside pid_update().
 *
 * @param[in]  pid   PID instance. Must not be NULL.
 * @param[out] terms PidTerms_t to populate. Must not be NULL.
 */
void pid_get_terms(const PID_t *pid, PidTerms_t *terms);

/**
 * @brief  Return true when the integral is saturated (at its limit).
 *
 * @details Indicates that the I term is hitting the anti-windup clamp.
 *          Useful during tuning: if the integral is always saturated,
 *          either i_limit is too low or Ki is too high for this plant.
 *
 * @param[in]  pid  PID instance. Must not be NULL.
 * @return bool  true when |integral| >= i_limit × 0.99.
 */
bool pid_is_saturated(const PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
