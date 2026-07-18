/**
 * @file    filters.h
 * @brief   Digital signal filters — public API.
 *
 * @details Provides four reusable filter types used throughout the
 *          micromouse firmware to smooth noisy sensor and speed signals.
 *
 *          WHY FILTERING IS NECESSARY
 *          ───────────────────────────
 *          Three noise sources dominate this robot:
 *
 *          1. ENCODER QUANTISATION
 *             840 counts/rev → 0.127 mm/count. At 200 mm/s the robot
 *             travels 0.2 mm/tick = 1.57 counts/tick. The fractional
 *             part appears as ±1 count noise on every delta read.
 *             Effect: PID derivative term spikes ±127 mm/s every tick.
 *             Fix: moving average on the D term (handled in pid.c),
 *             but also useful as a standalone filter here for display.
 *
 *          2. IR ADC NOISE
 *             Motor PWM switching (20 kHz, up to 5 A peak) couples into
 *             the analog supply through the 10 kΩ divider resistors.
 *             Residual noise ≈ ±20–40 ADC counts after differential
 *             measurement. For display and threshold logic this is fine.
 *             For the wall-centering PID error signal a low-pass filter
 *             reduces jitter without adding significant phase lag.
 *
 *          3. GYRO INTEGRATION DRIFT
 *             The MPU6500 zero-rate offset is calibrated at boot but
 *             drifts slowly with temperature. At room temperature the
 *             residual drift is ~0.02 deg/s → 0.02 deg per second of
 *             run time. Over an 8-minute qualifier run this accumulates
 *             to ~9.6 degrees — significant if not corrected. The
 *             gyro is not filtered (the IMU's DLPF handles that), but
 *             the integrated yaw is reset at every cell boundary by the
 *             wall-alignment routine to bound the drift.
 *
 *          FILTER TYPES PROVIDED
 *          ──────────────────────
 *
 *          LPF_t    — First-order IIR low-pass filter
 *            y[n] = α × x[n] + (1 − α) × y[n−1]
 *            α close to 1 = fast (little smoothing)
 *            α close to 0 = slow (heavy smoothing)
 *            Time constant: τ = −1 / (fs × ln(1−α))  where fs = sample rate
 *            Used by: battery.c (α=0.1), wall_follow.c (α=0.3)
 *
 *          MAV_t    — Moving average (N-tap FIR)
 *            y[n] = (1/N) × Σ x[n−k]  for k = 0..N−1
 *            Uniform weighting — all N samples equally weighted.
 *            Used by: PID D-term smoothing, IR display smoothing
 *            Default window: MAV_DEFAULT_N = 8 taps
 *
 *          EMA_t    — Exponential moving average (same math as IIR LPF,
 *            different API — alpha set from desired time constant in ms)
 *            Used when you want to specify "smooth over X milliseconds"
 *            rather than tuning α directly.
 *
 *          MEDIAN3  — Median of 3 consecutive samples
 *            Returns the middle value of {x[n−2], x[n−1], x[n]}.
 *            Removes isolated spike outliers (impulse noise) without
 *            the phase lag of averaging filters.
 *            Used by: IR sensor pre-processing to remove single-tick spikes.
 *
 *          ALL FILTERS ARE STATEFUL — one instance per signal.
 *          Initialise with xxx_init() before first use.
 *          Call xxx_update() exactly once per sample.
 *          Never share one filter instance between two signals.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h — CTRL_LOOP_HZ (for EMA time-constant conversion)
 *          <stdint.h>, <stdbool.h> only — no HAL, no hardware.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef FILTERS_H
#define FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * CONSTANTS
 * ======================================================================= */

/** Default number of taps for moving-average filter. */
#define MAV_DEFAULT_N       8U

/** Maximum allowed taps for moving-average filter (limits RAM per instance). */
#define MAV_MAX_N           32U

/* =========================================================================
 * TYPE 1 — FIRST-ORDER IIR LOW-PASS FILTER (LPF)
 * ======================================================================= */

/**
 * @brief  First-order IIR low-pass filter instance.
 *
 * @details y[n] = alpha × x[n] + (1 − alpha) × y[n−1]
 *          One multiply + one add per sample. Zero phase lag at DC.
 *          Phase lag at frequency f: −arctan(2πf/fc) where fc = cutoff.
 */
typedef struct
{
    float alpha;    /**< Smoothing factor [0.0, 1.0]. 1.0 = no filtering.  */
    float value;    /**< Current filter output (initialised to 0 or seed).  */
    bool  seeded;   /**< True after first update — avoids startup transient.*/
} LPF_t;

/**
 * @brief  Initialise an IIR low-pass filter.
 *
 * @details Sets alpha and zeros the output.  The first call to lpf_update()
 *          seeds the filter with the first sample (no startup transient).
 *
 * @param[out] f      Filter instance. Must not be NULL.
 * @param[in]  alpha  Smoothing factor [0.0, 1.0].
 *                    Recommended starting values:
 *                      0.05 – very heavy smoothing (battery voltage)
 *                      0.10 – heavy smoothing
 *                      0.30 – moderate smoothing (IR wall-centre error)
 *                      0.50 – light smoothing
 *                      1.00 – passthrough (no filtering)
 */
void  lpf_init(LPF_t *f, float alpha);

/**
 * @brief  Push a new sample and return the filtered value.
 *
 * @details On the first call, seeds the filter with the sample value
 *          so the output starts at the correct level immediately.
 *
 * @param[in,out] f    Filter instance.
 * @param[in]     x    New input sample.
 * @return float       Filtered output y[n].
 */
float lpf_update(LPF_t *f, float x);

/**
 * @brief  Return the current filter output without updating.
 *
 * @param[in] f  Filter instance.
 * @return float  Last filtered value.
 */
float lpf_value(const LPF_t *f);

/**
 * @brief  Reset filter output to a specific seed value.
 *
 * @details Use when you know the initial signal value and want to
 *          avoid the convergence transient (e.g. battery_init() seeds
 *          the battery filter with the first real reading).
 *
 * @param[in,out] f      Filter instance.
 * @param[in]     seed   Value to initialise the filter output with.
 */
void  lpf_seed(LPF_t *f, float seed);

/* =========================================================================
 * TYPE 2 — N-TAP MOVING AVERAGE (MAV)
 * ======================================================================= */

/**
 * @brief  N-tap moving average filter instance.
 *
 * @details Uniform FIR: y[n] = (1/N) × Σ x[n−k] for k=0..N−1.
 *          Uses a circular buffer and running sum for O(1) per update.
 *          Introduces N/2 taps of phase delay (N/2 ms at 1 kHz).
 */
typedef struct
{
    float    buf[MAV_MAX_N];  /**< Circular sample buffer.               */
    uint8_t  n;               /**< Number of taps (1 to MAV_MAX_N).      */
    uint8_t  idx;             /**< Next write position.                   */
    float    sum;             /**< Running sum of all buffer contents.    */
    uint8_t  count;           /**< Samples received (saturates at n).     */
} MAV_t;

/**
 * @brief  Initialise a moving-average filter.
 *
 * @details Zeros the buffer and running sum.
 *          The filter ramps up over the first n samples — output is the
 *          average of however many samples have been received so far.
 *
 * @param[out] m  Filter instance. Must not be NULL.
 * @param[in]  n  Number of taps [1, MAV_MAX_N].
 *               1 = passthrough.  8 = default.  32 = maximum.
 *               Values outside [1, MAV_MAX_N] are clamped silently.
 */
void  mav_init(MAV_t *m, uint8_t n);

/**
 * @brief  Push a new sample and return the current average.
 *
 * @param[in,out] m    Filter instance.
 * @param[in]     x    New input sample.
 * @return float       Average of the last n samples.
 */
float mav_update(MAV_t *m, float x);

/**
 * @brief  Return the current average without adding a new sample.
 *
 * @param[in] m  Filter instance.
 * @return float  Current moving average.
 */
float mav_value(const MAV_t *m);

/**
 * @brief  Reset buffer to all zeros.
 *
 * @param[in,out] m  Filter instance.
 */
void  mav_reset(MAV_t *m);

/* =========================================================================
 * TYPE 3 — EXPONENTIAL MOVING AVERAGE (EMA)
 * ======================================================================= */

/**
 * @brief  Exponential moving average — same math as IIR LPF but
 *         configured via a time-constant in milliseconds rather than α.
 *
 * @details α = 1 − exp(−Δt / τ)  where Δt = 1/CTRL_LOOP_HZ seconds.
 *          Useful when you think in terms of "smooth over 50 ms" rather
 *          than tuning a dimensionless α.
 *
 *          Examples at CTRL_LOOP_HZ = 1000 Hz (Δt = 1 ms):
 *            τ_ms =  10 ms → α = 1 − e^(−0.1) ≈ 0.095
 *            τ_ms =  50 ms → α = 1 − e^(−0.02)≈ 0.020
 *            τ_ms = 100 ms → α = 1 − e^(−0.01)≈ 0.010
 */
typedef struct
{
    float alpha;    /**< Computed from time constant at init.             */
    float value;    /**< Current EMA output.                              */
    bool  seeded;   /**< Startup seeding flag.                            */
} EMA_t;

/**
 * @brief  Initialise an EMA filter with a time constant in milliseconds.
 *
 * @param[out] e         EMA instance. Must not be NULL.
 * @param[in]  tau_ms    Time constant in milliseconds (> 0).
 *                       Larger = more smoothing, slower response.
 */
void  ema_init(EMA_t *e, float tau_ms);

/**
 * @brief  Push a new sample and return the EMA.
 *
 * @param[in,out] e  EMA instance.
 * @param[in]     x  New sample.
 * @return float     EMA output.
 */
float ema_update(EMA_t *e, float x);

/**
 * @brief  Return current EMA value without updating.
 */
float ema_value(const EMA_t *e);

/**
 * @brief  Seed the EMA with a known initial value.
 */
void  ema_seed(EMA_t *e, float seed);

/* =========================================================================
 * TYPE 4 — MEDIAN OF 3  (stateful, rolling window)
 * ======================================================================= */

/**
 * @brief  Stateful median-of-3 filter instance.
 *
 * @details Maintains a 3-sample history.  Each call to median3_update()
 *          pushes the new sample and returns the median of the last 3.
 *          Removes isolated single-tick spikes (impulse noise) without
 *          the smearing effect of a moving average.
 *          Phase delay: 1 sample (1 ms at 1 kHz).
 */
typedef struct
{
    float  buf[3]; /**< Last 3 samples (circular).                       */
    uint8_t idx;   /**< Next write position.                             */
    bool   ready;  /**< True after 3 samples received.                   */
} Median3_t;

/**
 * @brief  Initialise a median-of-3 filter.
 *
 * @param[out] m  Filter instance. Must not be NULL.
 */
void  median3_init(Median3_t *m);

/**
 * @brief  Push a new sample and return the median of the last 3.
 *
 * @details Before 3 samples have been received, returns the most
 *          recent sample (no output delay during startup).
 *
 * @param[in,out] m  Filter instance.
 * @param[in]     x  New sample.
 * @return float     Median of last 3 samples.
 */
float median3_update(Median3_t *m, float x);

/**
 * @brief  Stateless median of exactly 3 values.
 *
 * @details Returns the middle value of {a, b, c} with no state.
 *          Use when you already have 3 samples and just need the median.
 *
 * @param  a, b, c  Three float values (any order).
 * @return float    The median value.
 */
float median3_of(float a, float b, float c);

#ifdef __cplusplus
}
#endif

#endif /* FILTERS_H */
