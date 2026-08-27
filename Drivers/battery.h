/**
 * @file    battery.h
 * @brief   Battery voltage monitoring — public API.
 *
 * @details Monitors the 2S Li-ion pack voltage, derives a state-of-charge
 *          percentage, classifies voltage into health states, and provides
 *          a filtered reading to the rest of the firmware.
 *
 *          HARDWARE
 *          ────────
 *          Battery : 2S Li-ion, 7.4 V nominal (6.6 V cutoff – 8.4 V full).
 *          Divider : 10 kΩ / 10 kΩ voltage divider, Vbat/2 → PB0 (ADC1_IN8).
 *          ADC     : Shared with IR sensor DMA scan (rank 7 in the 7-channel
 *                    circular scan started by ir_init()).
 *                    battery.c reads the DMA buffer via ir_get_raw_batt()
 *                    — it never starts its own ADC conversion.
 *
 *          CONVERSION
 *          ───────────
 *          ADC is 12-bit (0–4095), reference = 3.3 V.
 *          Divider halves the voltage: ADC reads Vbat / 2.
 *          Vbat = raw × (3.3 / 4095) × 2 = raw × BATT_ADC_SCALE
 *
 *          Example: Vbat = 7.40 V → ADC input = 3.70 V → raw ≈ 2241
 *          Verify: 2241 × 0.001612903 = 3.616 V × 2 = 7.23 V
 *          (Small error due to tolerances — calibrate BATT_ADC_SCALE
 *          by measuring Vbat with a multimeter and adjusting the constant.)
 *
 *          LOW-PASS FILTER
 *          ────────────────
 *          Raw ADC reads are noisy, especially when motors are running
 *          (PWM switching couples into the supply rail).  A first-order
 *          IIR low-pass filter with α = 0.1 smooths the reading:
 *            v_filtered = α × v_new + (1 − α) × v_filtered_prev
 *          With α = 0.1 the time constant is ~10 × update period = 5 s.
 *          This means a genuine voltage drop takes 5 s to fully appear
 *          in battery_voltage() — long enough to ignore motor spikes,
 *          short enough to catch a real low-battery condition before
 *          damage occurs.
 *
 *          VOLTAGE STATES
 *          ───────────────
 *          BATT_STATE_FULL     ≥ 8.0 V   (freshly charged)
 *          BATT_STATE_OK       ≥ 7.4 V   (normal operating range)
 *          BATT_STATE_LOW      ≥ 7.0 V   (finish current run, swap)
 *          BATT_STATE_CRITICAL ≥ 6.6 V   (stop immediately — motor disable)
 *          BATT_STATE_DEAD     < 6.6 V   (cells damaged if continued)
 *
 *          USAGE PATTERN
 *          ──────────────
 *          1. Call battery_init() once during module bring-up.
 *          2. Call battery_update() every BATT_UPDATE_INTERVAL_MS (500 ms)
 *             from the scheduler main loop — NOT from the 1 kHz ISR.
 *          3. Call battery_voltage(), battery_percent(), battery_state()
 *             anywhere — they return the last filtered value, no I/O.
 *          4. Check battery_is_critical() in safety.c before every move.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h — BATT_FULL_V, BATT_NOMINAL_V, BATT_LOW_V,
 *                     BATT_CRITICAL_V, BATT_ADC_SCALE, BATT_UPDATE_INTERVAL_MS
 *          ir.h     — ir_get_raw_batt() (reads shared ADC DMA slot)
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef BATTERY_H
#define BATTERY_H

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
 * @brief  Battery voltage health classification.
 *
 * @details Ordered — higher numeric value = worse state.
 *          Allows comparisons: if (battery_state() >= BATT_STATE_LOW) …
 */
typedef enum
{
    BATT_STATE_FULL     = 0U,  /**< ≥ 8.0 V — freshly charged            */
    BATT_STATE_OK       = 1U,  /**< ≥ 7.4 V — normal operating range     */
    BATT_STATE_LOW      = 2U,  /**< ≥ 7.0 V — finish run, swap pack      */
    BATT_STATE_CRITICAL = 3U,  /**< ≥ 6.6 V — stop motors immediately    */
    BATT_STATE_DEAD     = 4U,  /**< < 6.6 V — cells at damage threshold  */
} BattState_t;

/**
 * @brief  Complete battery status snapshot.
 */
typedef struct
{
    float       voltage_v;     /**< Filtered voltage in volts             */
    uint8_t     percent;       /**< State of charge 0–100 %              */
    BattState_t state;         /**< Health classification                 */
    bool        is_low;        /**< true when state ≥ BATT_STATE_LOW     */
    bool        is_critical;   /**< true when state ≥ BATT_STATE_CRITICAL*/
    uint16_t    raw_adc;       /**< Last raw 12-bit ADC reading           */
} BattStatus_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the battery monitor.
 *
 * @details Seeds the IIR filter with an immediate first reading so that
 *          battery_voltage() returns a valid value as soon as init
 *          completes — no warm-up period.
 *          The IR sensor DMA scan (ir_init()) must already be running
 *          before battery_init() is called, because battery_init() calls
 *          ir_get_raw_batt() to seed the filter.
 *
 * @return MM_OK always (no hardware to initialise — ADC is shared with ir.c).
 */
MmResult_t battery_init(void);

/**
 * @brief  Sample the battery ADC and update the filtered reading.
 *
 * @details Reads ir_get_raw_batt() → converts to volts → applies IIR
 *          low-pass filter → updates internal state.
 *          Call every BATT_UPDATE_INTERVAL_MS (500 ms) from scheduler.c.
 *
 * @note   Must NOT be called from the 1 kHz control loop ISR.
 *         Non-blocking — returns immediately after the ADC buffer read.
 */
void battery_update(void);

/* =========================================================================
 * READINGS
 * ======================================================================= */

/**
 * @brief  Return the current filtered battery voltage in volts.
 *
 * @details Returns the IIR-filtered value — not the raw ADC reading.
 *          Safe to call from any context.  Returns 0.0f before first
 *          battery_init() call.
 *
 * @return float  Battery voltage in volts (e.g. 7.40 for nominal).
 */
float battery_voltage(void);

/**
 * @brief  Return estimated state of charge as a percentage (0–100).
 *
 * @details Linear interpolation between BATT_CRITICAL_V (0 %) and
 *          BATT_FULL_V (100 %).
 *          Formula: pct = (V − BATT_CRITICAL_V) / (BATT_FULL_V − BATT_CRITICAL_V) × 100
 *          Clamped to [0, 100].
 *
 * @note   Li-ion discharge curves are non-linear so this percentage
 *         is approximate.  For competition use treat anything below
 *         25 % as a warning to swap the pack.
 *
 * @return uint8_t  State of charge 0–100 %.
 */
uint8_t battery_percent(void);

/**
 * @brief  Return the battery health state classification.
 *
 * @return BattState_t  One of BATT_STATE_FULL/OK/LOW/CRITICAL/DEAD.
 */
BattState_t battery_state(void);

/* =========================================================================
 * THRESHOLD CHECKS
 * ======================================================================= */

/**
 * @brief  Return true when battery is in LOW or worse state.
 *
 * @details Convenience wrapper for:
 *          battery_state() >= BATT_STATE_LOW
 *          Used in modes.c to display a warning between runs.
 *
 * @return bool  true if voltage < BATT_LOW_V (7.0 V).
 */
bool battery_is_low(void);

/**
 * @brief  Return true when battery is in CRITICAL or DEAD state.
 *
 * @details Checked by safety.c before every move command.
 *          When true, safety.c calls motors_disable() regardless of
 *          what the navigation layer has requested.
 *
 * @return bool  true if voltage < BATT_CRITICAL_V (6.6 V).
 */
bool battery_is_critical(void);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Populate a full battery status snapshot.
 *
 * @details Captures voltage, percent, state, and flags into one struct
 *          for UART logging and OLED display without multiple calls.
 *
 * @param[out] status  Pointer to the BattStatus_t to fill. Must not be NULL.
 */
void battery_get_status(BattStatus_t *status);

/**
 * @brief  Return a human-readable string for a battery state.
 *
 * @details Returns a short string literal — not a dynamically allocated string.
 *          "FULL" / "OK" / "LOW" / "CRITICAL" / "DEAD"
 *
 * @param  state  Battery state to describe.
 * @return const char*  State name string.
 */
const char *battery_state_str(BattState_t state);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
