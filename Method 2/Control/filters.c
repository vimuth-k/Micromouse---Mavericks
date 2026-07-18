/**
 * @file    filters.c
 * @brief   Digital signal filters — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          Implements four reusable digital filters used across the
 *          firmware to clean up noisy signals before they reach PID
 *          controllers, display functions, and threshold comparisons.
 *
 *          WHERE EACH FILTER IS USED IN THIS PROJECT
 *          ──────────────────────────────────────────
 *
 *          LPF (IIR first-order):
 *            battery.c       α=0.1   smooths battery voltage over ~4.5s
 *            wall_follow.c   α=0.3   smooths IR side-error for wall PID
 *            diagnostics.c   α=0.5   smooths speed display values
 *
 *          MAV (moving average):
 *            pid.c           n=4     smooths PID derivative term
 *            modes.c         n=8     smooths sensor values for UART output
 *
 *          EMA (exponential moving average):
 *            imu.c           τ=5ms   smooths gyro Z rate before integration
 *            explorer.c      τ=20ms  smooths encoder speed display
 *
 *          MEDIAN3:
 *            ir.c            pre-processing: removes single-tick IR spikes
 *            battery.c       optional: removes ADC glitches before LPF
 *
 *          IMPLEMENTATION CHOICES
 *          ──────────────────────
 *
 *          LPF: y = α×x + (1−α)×y_prev
 *            Two float multiplies + one add per sample.  Fastest option.
 *            Seeding on first call avoids a convergence transient that
 *            would otherwise take τ/(Δt) ticks to reach the true value.
 *
 *          MAV: running sum with circular buffer
 *            Naive implementation would sum N values every tick: O(N).
 *            This implementation maintains a running sum: subtract the
 *            oldest value, add the newest.  O(1) regardless of N.
 *            Startup: during the first N calls, divides by actual count
 *            (not N) so the output is meaningful immediately.
 *
 *          EMA: same math as LPF, different initialisation
 *            α is computed from the time constant τ:
 *              α = 1 − exp(−Δt / τ)  where Δt = 1 ms
 *            expf() is called once at init — not per sample.
 *
 *          MEDIAN3: sorting network for exactly 3 elements
 *            A sorting network is a fixed sequence of compare-and-swap
 *            operations that sorts N elements in O(N log N) compare
 *            operations with no branches based on data values.
 *            For N=3 the optimal network uses 3 comparisons:
 *              compare(0,1), compare(0,2), compare(1,2)
 *            After these three operations, the middle element (index 1)
 *            holds the median.  No conditional branches on the data —
 *            compiles to efficient branchless code on Cortex-M4.
 *
 * @author  VDawn
 * @date    2026
 */

#include "filters.h"
#include "config.h"
#include <string.h>
#include <math.h>

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Swap two floats if a > b (used in sorting network).
 */
static inline void swap_if_gt(float *a, float *b)
{
    if (*a > *b)
    {
        float tmp = *a;
        *a = *b;
        *b = tmp;
    }
}

/* =========================================================================
 * TYPE 1 — IIR LOW-PASS FILTER (LPF)
 * ======================================================================= */

/**
 * @brief  Initialise an IIR low-pass filter.
 *
 * @details Clamps alpha to [0.0, 1.0].  Zeros the output.
 *          The seeded flag is false so the first update() call seeds
 *          the filter with the first sample rather than blending with 0.
 */
void lpf_init(LPF_t *f, float alpha)
{
    if (f == NULL) { return; }
    f->alpha  = (alpha < 0.0f) ? 0.0f : (alpha > 1.0f) ? 1.0f : alpha;
    f->value  = 0.0f;
    f->seeded = false;
}

/**
 * @brief  Push a new sample and return the filtered value.
 *
 * @details y[n] = α × x[n] + (1 − α) × y[n−1]
 *
 *          First-call seeding:
 *          Without seeding, if the filter starts at 0 and the true signal
 *          is 7.4 V (battery), the output would ramp from 0 → 7.4 V
 *          over τ seconds — giving false low-battery readings at boot.
 *          Seeding loads y[−1] = x[0] so the first output is exactly x[0].
 */
float lpf_update(LPF_t *f, float x)
{
    if (!f->seeded)
    {
        f->value  = x;
        f->seeded = true;
        return x;
    }
    f->value = (f->alpha * x) + ((1.0f - f->alpha) * f->value);
    return f->value;
}

float lpf_value(const LPF_t *f)
{
    return (f != NULL) ? f->value : 0.0f;
}

void lpf_seed(LPF_t *f, float seed)
{
    if (f == NULL) { return; }
    f->value  = seed;
    f->seeded = true;
}

/* =========================================================================
 * TYPE 2 — N-TAP MOVING AVERAGE (MAV)
 * ======================================================================= */

/**
 * @brief  Initialise a moving-average filter.
 *
 * @details Clamps n to [1, MAV_MAX_N].  Zeros buffer and running sum.
 */
void mav_init(MAV_t *m, uint8_t n)
{
    if (m == NULL) { return; }
    m->n     = (n < 1U) ? 1U : (n > MAV_MAX_N) ? (uint8_t)MAV_MAX_N : n;
    m->idx   = 0U;
    m->sum   = 0.0f;
    m->count = 0U;
    (void)memset(m->buf, 0, sizeof(m->buf));
}

/**
 * @brief  Push a new sample and return the moving average.
 *
 * @details Running-sum O(1) implementation:
 *          1. Subtract the oldest value from the sum.
 *          2. Write the new value at the current index.
 *          3. Add the new value to the sum.
 *          4. Advance the circular index.
 *          5. Divide sum by number of valid samples.
 *
 *          During startup (count < n), divides by count rather than n
 *          so the output is the true average of received samples.
 */
float mav_update(MAV_t *m, float x)
{
    if (m == NULL) { return x; }

    /* Remove oldest sample from the running sum */
    m->sum -= m->buf[m->idx];

    /* Write new sample */
    m->buf[m->idx] = x;
    m->sum        += x;

    /* Advance circular index */
    m->idx = (uint8_t)((m->idx + 1U) % m->n);

    /* Track fill level for startup averaging */
    if (m->count < m->n) { m->count++; }

    return m->sum / (float)m->count;
}

float mav_value(const MAV_t *m)
{
    if (m == NULL || m->count == 0U) { return 0.0f; }
    return m->sum / (float)m->count;
}

void mav_reset(MAV_t *m)
{
    if (m == NULL) { return; }
    m->idx   = 0U;
    m->sum   = 0.0f;
    m->count = 0U;
    (void)memset(m->buf, 0, sizeof(m->buf));
}

/* =========================================================================
 * TYPE 3 — EXPONENTIAL MOVING AVERAGE (EMA)
 * ======================================================================= */

/**
 * @brief  Initialise an EMA filter from a time constant in milliseconds.
 *
 * @details α = 1 − exp(−Δt / τ)
 *          where Δt = 1 / CTRL_LOOP_HZ = 0.001 s = 1 ms.
 *
 *          expf() is called once here — not on every sample.
 *
 *          Derivation of the formula:
 *          The time constant τ of a first-order IIR with coefficient α is:
 *            τ = −Δt / ln(1 − α)
 *          Solving for α:
 *            ln(1 − α) = −Δt / τ
 *            1 − α     = exp(−Δt / τ)
 *            α         = 1 − exp(−Δt / τ)
 *
 *          Example: τ = 50 ms, Δt = 1 ms
 *            α = 1 − exp(−0.02) = 1 − 0.9802 = 0.0198
 *            Each sample contributes 1.98 % of the output — heavy smoothing.
 */
void ema_init(EMA_t *e, float tau_ms)
{
    if (e == NULL) { return; }
    float dt_s = 1.0f / (float)CTRL_LOOP_HZ;   /* 0.001 s */
    float tau_s = (tau_ms > 0.0f) ? (tau_ms / 1000.0f) : 0.001f;
    e->alpha  = 1.0f - expf(-dt_s / tau_s);
    e->value  = 0.0f;
    e->seeded = false;
}

/**
 * @brief  Push a new sample and return the EMA.
 *
 * @details Same math as lpf_update() — seeded on first call.
 */
float ema_update(EMA_t *e, float x)
{
    if (e == NULL) { return x; }
    if (!e->seeded)
    {
        e->value  = x;
        e->seeded = true;
        return x;
    }
    e->value = (e->alpha * x) + ((1.0f - e->alpha) * e->value);
    return e->value;
}

float ema_value(const EMA_t *e)
{
    return (e != NULL) ? e->value : 0.0f;
}

void ema_seed(EMA_t *e, float seed)
{
    if (e == NULL) { return; }
    e->value  = seed;
    e->seeded = true;
}

/* =========================================================================
 * TYPE 4 — MEDIAN OF 3
 * ======================================================================= */

/**
 * @brief  Initialise a stateful median-of-3 filter.
 */
void median3_init(Median3_t *m)
{
    if (m == NULL) { return; }
    m->buf[0] = 0.0f;
    m->buf[1] = 0.0f;
    m->buf[2] = 0.0f;
    m->idx    = 0U;
    m->ready  = false;
}

/**
 * @brief  Push a new sample, return median of last 3.
 *
 * @details Maintains a 3-element circular buffer.  On each call:
 *          1. Write new sample into the buffer.
 *          2. Copy all 3 to a local array (preserves buffer).
 *          3. Apply 3-compare sorting network to the local copy.
 *          4. Return the middle element.
 *
 *          Before 3 samples have been received, returns the most
 *          recent sample to avoid startup latency.
 *
 *          SORTING NETWORK for N=3 (3 compare-and-swap operations):
 *            Step 1: swap_if_gt(t[0], t[1])   ensures t[0] ≤ t[1]
 *            Step 2: swap_if_gt(t[0], t[2])   ensures t[0] = min(all three)
 *            Step 3: swap_if_gt(t[1], t[2])   ensures t[1] = median, t[2] = max
 *          Return t[1].
 *
 *          This is optimal — no sorting network for N=3 can do it in fewer
 *          than 3 comparisons.
 */
float median3_update(Median3_t *m, float x)
{
    if (m == NULL) { return x; }

    /* Write new sample into circular buffer */
    m->buf[m->idx] = x;
    m->idx = (uint8_t)((m->idx + 1U) % 3U);

    /* Track startup */
    if (!m->ready)
    {
        /* After 2nd sample, idx wraps, we have at least 2 — but wait for 3 */
        if (m->idx == 0U) { m->ready = true; }
        else              { return x; }   /* Not enough samples yet */
    }

    /* Copy buffer for sorting (do not modify the ring buffer) */
    float t[3] = { m->buf[0], m->buf[1], m->buf[2] };

    /* 3-compare optimal sorting network */
    swap_if_gt(&t[0], &t[1]);
    swap_if_gt(&t[0], &t[2]);
    swap_if_gt(&t[1], &t[2]);

    return t[1];   /* median */
}

/**
 * @brief  Stateless median of exactly 3 values.
 *
 * @details Same sorting network as median3_update() but on caller-supplied
 *          values with no state involved.
 */
float median3_of(float a, float b, float c)
{
    /* Sort in-place using optimal 3-comparison network */
    swap_if_gt(&a, &b);
    swap_if_gt(&a, &c);
    swap_if_gt(&b, &c);
    return b;   /* median is now in b */
}
