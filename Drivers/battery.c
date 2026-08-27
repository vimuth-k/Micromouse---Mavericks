/**
 * @file    battery.c
 * @brief   Battery voltage monitoring — implementation.
 * @author  VDawn
 * @date    2026
 */

#include "battery.h"
#include "ir.h"
#include "config.h"

/* IIR low-pass filter coefficient (α = 0.1 → τ ≈ 4.5 s at 500 ms rate). */
#define BATT_FILTER_ALPHA   0.1f

/* Hysteresis band to prevent state bouncing at threshold boundaries (V). */
#define BATT_HYSTERESIS_V   0.10f

static float        s_voltage_v   = 0.0f;
static uint16_t     s_raw_adc     = 0U;
static BattState_t  s_state       = BATT_STATE_OK;
static bool         s_initialised = false;

/* -------------------------------------------------------------------------
 * PRIVATE HELPERS
 * ----------------------------------------------------------------------- */

static float raw_to_volts(uint16_t raw)
{
    return (float)raw * BATT_ADC_SCALE;
}

static void apply_filter(float new_v)
{
    s_voltage_v = (BATT_FILTER_ALPHA * new_v)
                + ((1.0f - BATT_FILTER_ALPHA) * s_voltage_v);
}

/**
 * Classify filtered voltage into BattState_t with 100 mV hysteresis.
 * Transitions DOWN (better→worse) occur at the exact threshold.
 * Transitions UP  (worse→better) require threshold + BATT_HYSTERESIS_V.
 */
static BattState_t classify_state(float v)
{
    switch (s_state)
    {
        case BATT_STATE_DEAD:
            if (v >= BATT_CRITICAL_V + BATT_HYSTERESIS_V)
                return BATT_STATE_CRITICAL;
            return BATT_STATE_DEAD;

        case BATT_STATE_CRITICAL:
            if (v < BATT_CRITICAL_V)
                return BATT_STATE_DEAD;
            if (v >= BATT_LOW_V + BATT_HYSTERESIS_V)
                return BATT_STATE_LOW;
            return BATT_STATE_CRITICAL;

        case BATT_STATE_LOW:
            if (v < BATT_CRITICAL_V)
                return BATT_STATE_DEAD;
            if (v < BATT_LOW_V)
                return BATT_STATE_CRITICAL;
            if (v >= BATT_NOMINAL_V + BATT_HYSTERESIS_V)
                return BATT_STATE_OK;
            return BATT_STATE_LOW;

        case BATT_STATE_OK:
            if (v < BATT_CRITICAL_V)
                return BATT_STATE_DEAD;
            if (v < BATT_LOW_V)
                return BATT_STATE_CRITICAL;
            if (v < BATT_NOMINAL_V)
                return BATT_STATE_LOW;
            if (v >= BATT_FULL_V + BATT_HYSTERESIS_V)
                return BATT_STATE_FULL;
            return BATT_STATE_OK;

        case BATT_STATE_FULL:
        default:
            if (v < BATT_CRITICAL_V)
                return BATT_STATE_DEAD;
            if (v < BATT_FULL_V - BATT_HYSTERESIS_V)
                return BATT_STATE_OK;
            return BATT_STATE_FULL;
    }
}

static uint8_t compute_percent(float v)
{
    float range = BATT_FULL_V - BATT_CRITICAL_V;
    float pct   = (v - BATT_CRITICAL_V) / range * 100.0f;
    if (pct > 100.0f) { pct = 100.0f; }
    if (pct <   0.0f) { pct =   0.0f; }
    return (uint8_t)pct;
}

/* -------------------------------------------------------------------------
 * PUBLIC API — LIFECYCLE
 * ----------------------------------------------------------------------- */

MmResult_t battery_init(void)
{
    s_raw_adc     = ir_get_raw_batt();
    s_voltage_v   = raw_to_volts(s_raw_adc);
    s_state       = classify_state(s_voltage_v);
    s_initialised = true;
    return MM_OK;
}

/* -------------------------------------------------------------------------
 * PUBLIC API — UPDATE
 * ----------------------------------------------------------------------- */

void battery_update(void)
{
    if (!s_initialised) { return; }
    s_raw_adc = ir_get_raw_batt();
    apply_filter(raw_to_volts(s_raw_adc));
    s_state = classify_state(s_voltage_v);
}

/* -------------------------------------------------------------------------
 * PUBLIC API — READINGS
 * ----------------------------------------------------------------------- */

float battery_voltage(void)
{
    return s_voltage_v;
}

uint8_t battery_percent(void)
{
    return compute_percent(s_voltage_v);
}

BattState_t battery_state(void)
{
    return s_state;
}

/* -------------------------------------------------------------------------
 * PUBLIC API — THRESHOLD CHECKS
 * ----------------------------------------------------------------------- */

bool battery_is_low(void)
{
    return s_state >= BATT_STATE_LOW;
}

bool battery_is_critical(void)
{
    return s_state >= BATT_STATE_CRITICAL;
}

/* -------------------------------------------------------------------------
 * PUBLIC API — DIAGNOSTICS
 * ----------------------------------------------------------------------- */

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

const char *battery_state_str(BattState_t state)
{
    static const char *const STRS[5] = {
        "FULL", "OK", "LOW", "CRITICAL", "DEAD"
    };
    return ((uint8_t)state < 5U) ? STRS[(uint8_t)state] : "UNKNOWN";
}
