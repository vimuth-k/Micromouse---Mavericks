/**
 * @file    battery.h
 * @brief   Battery voltage monitoring — public API.
 *
 * Monitors the 2S Li-ion pack (6.6–8.4 V) via a 10k/10k voltage divider
 * on PB0 (ADC1_IN8, rank 7 of the shared IR DMA scan). Applies an IIR
 * low-pass filter (α = 0.1, τ ≈ 4.5 s) to suppress PWM-induced ripple.
 *
 * Usage:
 *   1. battery_init()  — call once after ir_init() starts the ADC DMA.
 *   2. battery_update() — call every BATT_UPDATE_INTERVAL_MS (500 ms) from scheduler.
 *   3. battery_voltage() / battery_percent() / battery_state() — read anywhere.
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

/* -------------------------------------------------------------------------
 * TYPES
 * ----------------------------------------------------------------------- */

/** Battery voltage health classification. Higher value = worse state. */
typedef enum
{
    BATT_STATE_FULL     = 0U,  /**< >= 8.4 V — freshly charged            */
    BATT_STATE_OK       = 1U,  /**< >= 7.4 V — normal operating range     */
    BATT_STATE_LOW      = 2U,  /**< >= 7.0 V — finish run, swap pack      */
    BATT_STATE_CRITICAL = 3U,  /**< >= 6.6 V — stop motors immediately    */
    BATT_STATE_DEAD     = 4U,  /**< <  6.6 V — cells at damage threshold  */
} BattState_t;

/** Complete battery status snapshot. */
typedef struct
{
    float       voltage_v;     /**< Filtered voltage in volts             */
    uint8_t     percent;       /**< State of charge 0–100 %              */
    BattState_t state;         /**< Health classification                 */
    bool        is_low;        /**< true when state >= BATT_STATE_LOW     */
    bool        is_critical;   /**< true when state >= BATT_STATE_CRITICAL*/
    uint16_t    raw_adc;       /**< Last raw 12-bit ADC reading           */
} BattStatus_t;

/* -------------------------------------------------------------------------
 * LIFECYCLE
 * ----------------------------------------------------------------------- */

/** Seed the IIR filter with first ADC reading. ir_init() must run first. */
MmResult_t battery_init(void);

/** Sample ADC and update filtered reading. Call every 500 ms from scheduler. */
void battery_update(void);

/* -------------------------------------------------------------------------
 * READINGS
 * ----------------------------------------------------------------------- */

/** Filtered battery voltage in volts. */
float battery_voltage(void);

/** Estimated state of charge 0–100 % (linear, approximate). */
uint8_t battery_percent(void);

/** Current health state classification. */
BattState_t battery_state(void);

/* -------------------------------------------------------------------------
 * THRESHOLD CHECKS
 * ----------------------------------------------------------------------- */

/** True when voltage < BATT_LOW_V (7.0 V). */
bool battery_is_low(void);

/** True when voltage < BATT_CRITICAL_V (6.6 V) — disable motors. */
bool battery_is_critical(void);

/* -------------------------------------------------------------------------
 * DIAGNOSTICS
 * ----------------------------------------------------------------------- */

/** Populate a full status snapshot (voltage, percent, state, flags). */
void battery_get_status(BattStatus_t *status);

/** Human-readable string for a state: "FULL"/"OK"/"LOW"/"CRITICAL"/"DEAD". */
const char *battery_state_str(BattState_t state);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
