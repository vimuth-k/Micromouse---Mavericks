/**
 * @file    safety.h
 * @brief   MicroMaze 3 · Safety watchdog — stall, run-timeout, distance,
 *          and yaw-error protection.
 * @details
 *   WHAT THIS MODULE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   safety_check() is registered with the scheduler at
 *   SCHED_SAFETY_INTERVAL_MS (50 ms, 20 Hz — see scheduler.c) and runs
 *   five independent checks every call, using constants from config.h
 *   Section 16:
 *
 *     1. Battery critical    → motors_disable(). Second line of defence
 *        behind battery.c's own check in scheduler.c's battery task —
 *        this one runs 10x more often (50 ms vs 500 ms).
 *     2. Motor stall          → motion_stop(). Measured wheel speed stays
 *        below SAFETY_STALL_SPD_MMPS for SAFETY_STALL_TIME_MS while PWM
 *        is above SAFETY_STALL_PWM_THRESH (the PWM qualifier avoids a
 *        false "stall" firing every time the robot is simply commanded
 *        to be stopped or coasting).
 *     3. Run timeout          → motion_stop(). Time since safety_run_start()
 *        exceeds SAFETY_MAX_RUN_MS (the 8-minute MicroMaze 3 trial limit).
 *     4. Max distance         → motion_stop(). Cumulative odometry since
 *        safety_run_start() exceeds SAFETY_MAX_DIST_MM — a runaway/
 *        infinite-loop guard independent of the maze solver's own logic.
 *     5. Yaw error             → motion_stop(). While driving straight
 *        (MOTION_FORWARD), |yaw_deg| exceeds SAFETY_MAX_YAW_DEG — the
 *        heading-hold PID in motion.c should never let this happen in
 *        normal operation, so tripping this means something is
 *        seriously wrong (wheel slip, gyro fault, physical obstruction)
 *        and PID correction cannot be trusted to recover on its own.
 *
 *   RUN LIFECYCLE
 *   ─────────────────────────────────────────────────────────────────────
 *   Checks 3 and 4 (run timeout, max distance) only make sense relative
 *   to a run's start. Call safety_run_start() once when a maze
 *   run/search/speedrun begins (from modes.c, not yet written) — this
 *   resets the run clock and the cumulative distance accumulator.
 *   Checks 1, 2, and 5 are always active regardless of run state.
 *
 *   TRIP STATE
 *   ─────────────────────────────────────────────────────────────────────
 *   When any check fires, safety.c records which one via
 *   safety_trip_reason() and latches safety_is_tripped() true. The
 *   latch persists — including across further safety_check() calls —
 *   until safety_clear() is called explicitly, so a state machine (not
 *   yet written) can detect the stop, react (display the reason, wait
 *   for operator acknowledgement), and clear it before the next run.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Reason the safety module most recently stopped the robot.
 */
typedef enum
{
    SAFETY_TRIP_NONE = 0U,        /**< No trip since last safety_clear(). */
    SAFETY_TRIP_BATTERY_CRITICAL, /**< Battery voltage below critical.    */
    SAFETY_TRIP_STALL,            /**< Motors driven but wheels not moving. */
    SAFETY_TRIP_RUN_TIMEOUT,      /**< SAFETY_MAX_RUN_MS exceeded.        */
    SAFETY_TRIP_MAX_DISTANCE,     /**< SAFETY_MAX_DIST_MM exceeded.       */
    SAFETY_TRIP_YAW_ERROR,        /**< SAFETY_MAX_YAW_DEG exceeded.       */
} SafetyTripReason_t;

/**
 * @brief  Initialise the safety module. Call once during system bring-up,
 *         after motors.c and motion.c are initialised.
 * @details Clears the trip latch and zeroes the run clock / distance
 *          accumulator / stall timer.
 * @return MM_OK always.
 */
MmResult_t safety_init(void);

/**
 * @brief  Mark the start of a timed run for the run-timeout and
 *         max-distance checks.
 * @details Call once when a search or speedrun begins. Resets the run
 *          clock (for SAFETY_MAX_RUN_MS) and the cumulative distance
 *          accumulator (for SAFETY_MAX_DIST_MM) to zero. Does not clear
 *          an existing trip latch — call safety_clear() separately if
 *          starting fresh after a previous trip.
 */
void safety_run_start(void);

/**
 * @brief  Run all five safety checks once. Registered with the
 *         scheduler at SCHED_SAFETY_INTERVAL_MS (50 ms) — not normally
 *         called directly.
 * @details Any check that fires calls motors_disable() and/or
 *          motion_stop() as appropriate,
 *          and latches the trip (see safety_is_tripped()).
 *          If a trip is already latched, subsequent calls still run all
 *          checks (so a worse condition can overwrite the recorded
 *          reason) but do not re-issue the stop commands redundantly
 *          beyond what motion.c/motors.c already treat as idempotent.
 */
void safety_check(void);

/**
 * @brief  Has a safety check tripped since the last safety_clear()?
 * @return true if any check has fired and not yet been cleared.
 */
bool safety_is_tripped(void);

/**
 * @brief  Which check most recently tripped.
 * @return SAFETY_TRIP_NONE if nothing has tripped since the last
 *         safety_clear(), otherwise the reason for the latched trip.
 */
SafetyTripReason_t safety_trip_reason(void);

/**
 * @brief  Clear the trip latch so safety_is_tripped() returns false
 *         again. Call after the operator has acknowledged the stop and
 *         before starting a new run.
 * @note   Does not re-enable motors or resume motion — that remains an
 *         explicit decision for the caller (motors_enable() / the next
 *         motion command).
 */
void safety_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_H */
