/**
 * @file    battery.c
 * @brief   Battery voltage monitoring — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *
 *          1. READS THE SHARED ADC DMA SLOT
 *             ir.c runs a 7-channel ADC1 DMA circular scan.  Slot 6
 *             (IR_IDX_BATT, PB0, ADC1_IN8) is reserved for the battery
 *             voltage divider.  battery.c reads this slot via
 *             ir_get_raw_batt() — it never starts its own conversion,
 *             never touches hadc1, and never conflicts with ir.c.
 *             This is the "free battery reading" design: one DMA scan
 *             feeds both the IR pipeline and the battery monitor.
 *
 *          2. IIR LOW-PASS FILTER
 *             Motor PWM switching (20 kHz, up to 5 A peak) creates
 *             voltage ripple on the battery rail that reaches the ADC
 *             through the 10 kΩ / 10 kΩ divider.  Without filtering,
 *             battery_voltage() would oscillate by ±0.3 V in sync with
 *             the PWM frequency, which would constantly trigger and clear
 *             the low-battery warning.
 *
 *             The IIR filter with α = BATT_FILTER_ALPHA (0.1) gives a
 *             time constant of:
 *               τ = update_interval × (1 / α − 1) = 0.5 s × 9 = 4.5 s
 *             Motor current spikes shorter than ~1 s are attenuated by
 *             ≥ 80 % before reaching battery_voltage().
 *
 *             α is a compile-time constant — change it in this file
 *             if your battery rail is noisier or you need faster response.
 *
 *          3. STATE CLASSIFICATION
 *             After filtering, classify_state() maps the voltage to a
 *             BattState_t enum using the thresholds from config.h.
 *             The state machine uses 100 mV hysteresis to prevent
 *             bouncing at threshold boundaries:
 *               Transition DOWN (OK → LOW):     voltage < BATT_LOW_V
 *               Transition UP   (LOW → OK):     voltage > BATT_LOW_V + 0.1
 *             This prevents the buzzer pattern triggering on and off
 *             every 500 ms when the voltage hovers near 7.0 V.
 *
 *          4. PERCENTAGE CALCULATION
 *             Linear interpolation between BATT_CRITICAL_V (0%) and
 *             BATT_FULL_V (100%).  Li-ion discharge is not linear but
 *             this gives a useful approximation for competition use.
 *             Most of the usable capacity is between 7.0 and 8.0 V —
 *             treat readings below 25 % as a swap warning.
 *
 *          5. INITIALISATION SEEDING
 *             battery_init() takes an immediate first ADC reading and
 *             loads it directly into the filter state (not averaged over
 *             a warmup period).  This means battery_voltage() returns a
 *             valid reading immediately after init — no delay waiting for
 *             the filter to converge from zero.
 *
 * @author  VDawn
 * @date    2026
 */

#include "battery.h"
#include "ir.h"
#include "config.h"
#include "utils.h"

/* =========================================================================
 * PRIVATE CONSTANTS
 * ======================================================================= */

/**
 * @brief  IIR low-pass filter coefficient.
 *
 * @details α = 0.1 gives a time constant of ~4.5 s at 500 ms update rate.
 *          Increase toward 1.0 for faster response (less filtering).
 *          Decrease toward 0.0 for more filtering (slower response).
 */
#define BATT_FILTER_ALPHA       0.1f

/**
 * @brief  Hysteresis band for state transitions (volts).
 *
 * @details Prevents bouncing at threshold boundaries.
 *          A state transition DOWN triggers at the threshold.
 *          A transition UP requires voltage > threshold + hysteresis.
 */
#define BATT_HYSTERESIS_V       0.10f

/** Voltage mapped to 0 % (below this = cells at damage risk). */
#define BATT_PCT_ZERO_V         BATT_CRITICAL_V

/** Voltage mapped to 100 % (above this = fully charged). */
#define BATT_PCT_FULL_V         BATT_FULL_V

/* =========================================================================
 * PRIVATE STATE
 * ======================================================================= */

/** IIR-filtered voltage — the primary output of this module. */
static float        s_voltage_v     = 0.0f;

/** Last raw 12-bit ADC reading (for diagnostics). */
static uint16_t     s_raw_adc       = 0U;

/** Current classified health state. */
static BattState_t  s_state         = BATT_STATE_OK;

/** True after battery_init() has been called. */
static bool         s_initialised   = false;

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Convert a raw 12-bit ADC count to battery voltage in volts.
 *
 * @details Applies BATT_ADC_SCALE from config.h:
 *          Vbat = raw × (3.3 / 4095) × 2
 *          The ×2 factor accounts for the 10k/10k voltage divider.
 *
 * @param  raw  12-bit ADC count (0–4095).
 * @return float  Battery voltage in volts.
 */
static float raw_to_volts(uint16_t raw)
{
    return (float)raw * BATT_ADC_SCALE;
}

/**
 * @brief  Apply IIR low-pass filter to a new voltage sample.
 *
 * @details y[n] = α × x[n] + (1 − α) × y[n−1]
 *          Updates s_voltage_v in place.
 *
 * @param  new_v  New unfiltered voltage sample.
 */
static void apply_filter(float new_v)
{
    s_voltage_v = (BATT_FILTER_ALPHA * new_v)
                + ((1.0f - BATT_FILTER_ALPHA) * s_voltage_v);
}

/**
 * @brief  Classify filtered voltage into a BattState_t with hysteresis.
 *
 * @details Transitions DOWN (higher to worse state) happen at the
 *          threshold boundary.  Transitions UP (worse to better state)
 *          require the voltage to exceed the threshold by BATT_HYSTERESIS_V
 *          before the state improves.  This prevents rapid state flickering
 *          when voltage hovers near a boundary.
 *
 * @param  v  Filtered voltage in volts.
 * @return BattState_t  New classified state.
 */
static BattState_t classify_state(float v)
{
    /* Transitions DOWN: trigger at the exact threshold */
    if (v < BATT_CRITICAL_V)   { return BATT_STATE_DEAD;     }
    if (v < BATT_LOW_V)        { return BATT_STATE_CRITICAL;  }

    /* Apply hysteresis for upward transitions only */
    switch (s_state)
    {
        case BATT_STATE_DEAD:
            /* Recover from DEAD only if voltage climbs well above critical */
            if (v >= (BATT_CRITICAL_V + BATT_HYSTERESIS_V))
                return BATT_STATE_CRITICAL;
            return BATT_STATE_DEAD;

        case BATT_STATE_CRITICAL:
            if (v >= (BATT_LOW_V + BATT_HYSTERESIS_V))
                return BATT_STATE_LOW;
            if (v < BATT_CRITICAL_V)
                return BATT_STATE_DEAD;
            return BATT_STATE_CRITICAL;

        case BATT_STATE_LOW:
            if (v >= (BATT_NOMINAL_V + BATT_HYSTERESIS_V))
                return BATT_STATE_OK;
            if (v < BATT_LOW_V)
                return BATT_STATE_CRITICAL;
            return BATT_STATE_LOW;

        case BATT_STATE_OK:
            if (v >= (8.0f + BATT_HYSTERESIS_V))
                return BATT_STATE_FULL;
            if (v < BATT_LOW_V)
                return BATT_STATE_CRITICAL;
            if (v < BATT_NOMINAL_V)
                return BATT_STATE_LOW;
            return BATT_STATE_OK;

        case BATT_STATE_FULL:
        default:
            /* Drop from FULL if voltage falls below nominal */
            if (v < (8.0f - BATT_HYSTERESIS_V))
                return BATT_STATE_OK;
            return BATT_STATE_FULL;
    }
}

/**
 * @brief  Compute state of charge percentage from filtered voltage.
 *
 * @details Linear interpolation:
 *          pct = (V − zero_V) / (full_V − zero_V) × 100
 *          Clamped to [0, 100].
 *
 * @param  v  Filtered voltage.
 * @return uint8_t  Percentage 0–100.
 */
static uint8_t compute_percent(float v)
{
    float range = BATT_PCT_FULL_V - BATT_PCT_ZERO_V;   /* 8.4 - 6.6 = 1.8 V */
    float pct   = (v - BATT_PCT_ZERO_V) / range * 100.0f;

    if (pct > 100.0f) { pct = 100.0f; }
    if (pct <   0.0f) { pct =   0.0f; }

    return (uint8_t)pct;
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the battery monitor.
 *
 * @details Seeds the IIR filter with an immediate first reading.
 *          ir_init() must already have started the ADC DMA scan before
 *          this is called.
 */
MmResult_t battery_init(void)
{
    /* Seed the filter with the current ADC value so battery_voltage()
     * returns a valid reading immediately rather than starting at 0.0f
     * and ramping up over several update cycles.                        */
    s_raw_adc   = ir_get_raw_batt();
    s_voltage_v = raw_to_volts(s_raw_adc);
    s_state     = classify_state(s_voltage_v);
    s_initialised = true;

    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — UPDATE  (call every BATT_UPDATE_INTERVAL_MS from scheduler)
 * ======================================================================= */

/**
 * @brief  Sample battery ADC and update the filtered reading.
 *
 * @details Reads the latest raw ADC value from the IR module's shared
 *          DMA buffer, converts to volts, applies the IIR filter,
 *          and re-classifies the state.
 *          Non-blocking — returns in under 5 µs.
 */
void battery_update(void)
{
    if (!s_initialised) { return; }

    /* Read the latest value from the shared ADC DMA buffer */
    s_raw_adc = ir_get_raw_batt();

    /* Convert raw ADC count to volts */
    float new_v = raw_to_volts(s_raw_adc);

    /* Apply IIR low-pass filter */
    apply_filter(new_v);

    /* Reclassify state with hysteresis */
    s_state = classify_state(s_voltage_v);
}

/* =========================================================================
 * PUBLIC API — READINGS
 * ======================================================================= */

/**
 * @brief  Return filtered battery voltage in volts.
 */
float battery_voltage(void)
{
    return s_voltage_v;
}

/**
 * @brief  Return estimated state of charge percentage (0–100).
 */
uint8_t battery_percent(void)
{
    return compute_percent(s_voltage_v);
}

/**
 * @brief  Return current battery health state.
 */
BattState_t battery_state(void)
{
    return s_state;
}

/* =========================================================================
 * PUBLIC API — THRESHOLD CHECKS
 * ======================================================================= */

/**
 * @brief  True when battery needs swapping before the next run.
 */
bool battery_is_low(void)
{
    return s_state >= BATT_STATE_LOW;
}

/**
 * @brief  True when battery is critically low — disable motors now.
 */
bool battery_is_critical(void)
{
    return s_state >= BATT_STATE_CRITICAL;
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a full battery status snapshot.
 */
void battery_get_status(BattStatus_t *status)
{
    if (status == NULL) { return; }

    status->voltage_v   = s_voltage_v;
    status->percent     = compute_percent(s_voltage_v);
    status->state       = s_state;
    status->is_low      = battery_is_low();
    status->is_critical = battery_is_critical();
    status->raw_adc     = s_raw_adc;
}

/**
 * @brief  Return a human-readable string for a battery state.
 */
const char *battery_state_str(BattState_t state)
{
    static const char *const STATE_STRINGS[5] = {
        "FULL",     /* BATT_STATE_FULL     */
        "OK",       /* BATT_STATE_OK       */
        "LOW",      /* BATT_STATE_LOW      */
        "CRITICAL", /* BATT_STATE_CRITICAL */
        "DEAD",     /* BATT_STATE_DEAD     */
    };

    if ((uint8_t)state < 5U)
    {
        return STATE_STRINGS[(uint8_t)state];
    }
    return "UNKNOWN";
}
