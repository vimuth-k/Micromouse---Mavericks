/**
 * @file    pid.c
 * @brief   Generic discrete-time PID controller — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          Implements a single PID_t struct and five functions that
 *          operate on it.  The same code is instantiated five times
 *          in motion.c for the five independent control loops:
 *
 *          INSTANCE          ERROR SIGNAL                    OUTPUT
 *          ──────────────    ──────────────────────────────  ──────────────
 *          pid_speed_left    target_speed − meas_speed_L     PWM left
 *          pid_speed_right   target_speed − meas_speed_R     PWM right
 *          pid_straight      enc_left_count − enc_right_count correction
 *          pid_heading       0 − gyro_yaw (target=straight)  correction
 *          pid_turn          target_angle − gyro_yaw          turn PWM
 *
 *          DISCRETE IMPLEMENTATION DETAILS
 *          ─────────────────────────────────
 *
 *          A. TIME BASE
 *             All five loops run at 1 kHz (CTRL_LOOP_HZ) from the TIM5
 *             ISR.  The gains (Kp, Ki, Kd) are therefore in per-tick units,
 *             NOT per-second units.  When tuning from theory:
 *               Kp_tick  = Kp_continuous
 *               Ki_tick  = Ki_continuous × Δt  = Ki_s / 1000
 *               Kd_tick  = Kd_continuous / Δt  = Kd_s × 1000
 *             In practice: tune empirically starting with Ki=Kd=0,
 *             then increase Kp until oscillation appears, back off,
 *             add Ki to eliminate steady-state error, add Kd to dampen.
 *
 *          B. INTEGRAL ANTI-WINDUP — CLAMPING
 *             The integral accumulates error over time.  Without limiting
 *             it, a sustained error (e.g. robot stuck against a wall)
 *             causes the integral to grow without bound.  When the error
 *             eventually reverses, the large integral must "unwind" before
 *             the output responds — causing overshoot and oscillation.
 *             This module clamps the integral to [-i_limit, +i_limit]
 *             every tick.  Choose i_limit such that Ki × i_limit ≤
 *             a reasonable fraction of out_limit (≤30 % is a safe start).
 *
 *          C. DERIVATIVE TERM — ERROR DIFFERENCE
 *             D = Kd × (error[n] − error[n−1])
 *             We differentiate the ERROR, not the MEASUREMENT.
 *             Differentiating measurement is preferred in some literature
 *             (avoids D kick on setpoint step change) but for this robot
 *             setpoint steps are small (speed ramps from motion_profile.c,
 *             angle targets are single values set once per turn) and the
 *             encoder quantisation noise is the dominant D noise source.
 *             Error-based D is simpler and equivalent when setpoint
 *             changes are slow relative to the loop rate.
 *
 *          D. DERIVATIVE MOVING-AVERAGE FILTER
 *             Encoder resolution: 840 counts/rev, wheel circumference
 *             106.8 mm → 0.127 mm/count.  At 200 mm/s search speed the
 *             robot travels 0.2 mm per 1ms tick = ~1.57 counts/tick.
 *             The derivative (error difference) therefore jumps between
 *             0 and ±2–3 counts per tick just from quantisation.  With
 *             Kd = 0.05, this creates ±0.1–0.15 PWM variation per tick —
 *             small but audible as motor buzz.  A 4-tap moving average
 *             on the D term reduces this by 75 % with a phase lag of
 *             only 2 ms — negligible at 1 kHz.
 *             Enable by calling pid_init_ex() with d_filter_n = 4.
 *
 *          E. OUTPUT CLAMPING
 *             The final output is clamped to [-out_limit, +out_limit].
 *             For speed PIDs: out_limit = PWM_MAX (4999) — full range.
 *             For correction PIDs (straight, heading): out_limit is a
 *             fraction of PWM_MAX (30 %, 25 %) so the correction never
 *             overwhelms the speed command.
 *
 * @author  VDawn
 * @date    2026
 */

#include "pid.h"
#include <string.h>

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Symmetric clamp of x to [-limit, +limit].
 *
 * @param  x      Value to clamp.
 * @param  limit  Clamp magnitude (must be >= 0).
 * @return float  Clamped value.
 */
static float clamp_sym(float x, float limit)
{
    if (x >  limit) { return  limit; }
    if (x < -limit) { return -limit; }
    return x;
}

/**
 * @brief  Push a new value into the derivative moving-average buffer
 *         and return the current average.
 *
 * @details Maintains a running sum so the average is O(1) per tick —
 *          no loop over the buffer.  Handles the n=1 case (no filter)
 *          as a special path that returns d_raw directly.
 *
 * @param[in,out] pid    PID instance (owns the filter buffer).
 * @param[in]     d_raw  Raw derivative sample (error[n] − error[n−1]).
 * @return float  Filtered derivative value.
 */
static float d_filter_update(PID_t *pid, float d_raw)
{
    if (pid->d_filter_n <= 1U)
    {
        /* No filter — return raw derivative directly */
        return d_raw;
    }

    /* Remove the oldest sample from the running sum */
    pid->d_buf_sum -= pid->d_buf[pid->d_buf_idx];

    /* Write the new sample into the circular buffer */
    pid->d_buf[pid->d_buf_idx] = d_raw;
    pid->d_buf_sum += d_raw;

    /* Advance the write index (wrap at d_filter_n, not MAX_TAPS) */
    pid->d_buf_idx = (uint8_t)((pid->d_buf_idx + 1U) % pid->d_filter_n);

    /* Return the current average */
    return pid->d_buf_sum / (float)pid->d_filter_n;
}

/**
 * @brief  Zero all per-tick state fields within a PID_t.
 *
 * @details Shared by pid_init(), pid_init_ex(), and pid_reset().
 *          Does NOT touch gains, limits, or d_filter_n.
 *
 * @param[out] pid  PID instance to clear.
 */
static void clear_state(PID_t *pid)
{
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->d_buf_idx   = 0U;
    pid->d_buf_sum   = 0.0f;
    pid->last_p      = 0.0f;
    pid->last_i      = 0.0f;
    pid->last_d      = 0.0f;
    pid->last_output = 0.0f;
    (void)memset(pid->d_buf, 0, sizeof(pid->d_buf));
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise a PID instance with no D filter.
 */
void pid_init(PID_t *pid,
              float  kp, float ki, float kd,
              float  i_limit, float out_limit)
{
    pid_init_ex(pid, kp, ki, kd, i_limit, out_limit, 1U);
}

/**
 * @brief  Initialise a PID instance with an optional D-term filter.
 */
void pid_init_ex(PID_t  *pid,
                 float   kp, float ki, float kd,
                 float   i_limit, float out_limit,
                 uint8_t d_filter_n)
{
    if (pid == NULL) { return; }

    /* Gains */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /* Limits */
    pid->i_limit   = (i_limit   > 0.0f) ? i_limit   : 0.0f;
    pid->out_limit = (out_limit > 0.0f) ? out_limit : 0.0f;

    /* D filter — clamp to valid range */
    pid->d_filter_n = (d_filter_n < 1U) ? 1U :
                      (d_filter_n > PID_D_FILTER_MAX_TAPS)
                          ? (uint8_t)PID_D_FILTER_MAX_TAPS
                          : d_filter_n;

    /* Zero all state */
    clear_state(pid);
}

/**
 * @brief  Zero all per-tick state without changing gains or limits.
 */
void pid_reset(PID_t *pid)
{
    if (pid == NULL) { return; }
    clear_state(pid);
}

/* =========================================================================
 * PUBLIC API — RUNTIME
 * ======================================================================= */

/**
 * @brief  Compute one PID output step.
 *
 * @details Full annotated computation:
 *
 *          1. PROPORTIONAL
 *             p = Kp × error
 *             Linear response to current error.  The dominant term
 *             for fast corrections (turns, speed steps).
 *
 *          2. INTEGRAL
 *             integral += error              (accumulate)
 *             integral  = clamp(integral)   (anti-windup)
 *             i = Ki × integral
 *             Eliminates steady-state offset.  Without I, the speed PID
 *             would settle slightly below target because friction and
 *             back-EMF require a nonzero P term to maintain speed, which
 *             requires a nonzero error.  I removes this residual error.
 *
 *          3. DERIVATIVE (filtered)
 *             d_raw      = error − prev_error
 *             d_filtered = moving_average(d_raw, d_filter_n)
 *             d = Kd × d_filtered
 *             Responds to the rate of error change — damps oscillation.
 *             Without D, Kp can be pushed higher for faster response but
 *             the system overshoots.  D allows higher Kp while keeping
 *             the transient damped.
 *
 *          4. SUM AND CLAMP
 *             output = clamp(p + i + d, -out_limit, +out_limit)
 *
 *          5. STATE UPDATE
 *             prev_error = error  (ready for next tick's D computation)
 *
 *          6. DIAGNOSTICS
 *             last_p / last_i / last_d / last_output cached for display.
 */
float pid_update(PID_t *pid, float error)
{
    if (pid == NULL) { return 0.0f; }

    /* ── 1. Proportional ─────────────────────────────────────────── */
    float p = pid->kp * error;

    /* ── 2. Integral with anti-windup clamp ──────────────────────── */
    pid->integral += error;
    pid->integral  = clamp_sym(pid->integral, pid->i_limit);
    float i        = pid->ki * pid->integral;

    /* ── 3. Derivative with optional moving-average filter ────────── */
    float d_raw      = error - pid->prev_error;
    float d_filtered = d_filter_update(pid, d_raw);
    float d          = pid->kd * d_filtered;

    /* ── 4. Sum and clamp ─────────────────────────────────────────── */
    float raw_output = p + i + d;
    float output     = clamp_sym(raw_output, pid->out_limit);

    /* ── 5. State update ──────────────────────────────────────────── */
    pid->prev_error = error;

    /* ── 6. Cache for diagnostics ─────────────────────────────────── */
    pid->last_p      = p;
    pid->last_i      = i;
    pid->last_d      = d;
    pid->last_output = raw_output;   /* unclamped — shows saturation   */

    return output;
}

/* =========================================================================
 * PUBLIC API — GAIN ADJUSTMENT
 * ======================================================================= */

/**
 * @brief  Update all three gains at runtime without resetting state.
 */
void pid_set_gains(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) { return; }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief  Update the output clamp limit at runtime.
 */
void pid_set_output_limit(PID_t *pid, float out_limit)
{
    if (pid == NULL) { return; }
    pid->out_limit = (out_limit > 0.0f) ? out_limit : 0.0f;
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a PidTerms_t snapshot of the last computed step.
 */
void pid_get_terms(const PID_t *pid, PidTerms_t *terms)
{
    if (pid == NULL || terms == NULL) { return; }

    terms->p        = pid->last_p;
    terms->i        = pid->last_i;
    terms->d        = pid->last_d;
    terms->output   = pid->last_output;
    terms->error    = pid->prev_error;
    terms->integral = pid->integral;
}

/**
 * @brief  Return true when the integral is saturated at its limit.
 *
 * @details Uses a 1 % tolerance band (0.99 × i_limit) to avoid
 *          false negatives from floating-point rounding.
 */
bool pid_is_saturated(const PID_t *pid)
{
    if (pid == NULL)          { return false; }
    if (pid->i_limit <= 0.0f) { return false; }

    float abs_integral = (pid->integral < 0.0f)
                         ? -pid->integral
                         :  pid->integral;

    return abs_integral >= (pid->i_limit * 0.99f);
}
