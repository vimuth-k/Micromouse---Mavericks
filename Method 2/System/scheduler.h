/**
 * @file    scheduler.h
 * @brief   Cooperative task scheduler — public API.
 *
 * @details Manages the main-loop periodic tasks that must run at fixed
 *          intervals but do NOT belong in the 1 kHz ISR.
 *
 *          WHAT THE SCHEDULER IS NOT
 *          ──────────────────────────
 *          This is NOT a preemptive RTOS.  It does NOT run tasks in
 *          parallel.  It does NOT give hard real-time guarantees.
 *          It is a simple cooperative polling loop that calls each
 *          registered task whenever its interval has elapsed.
 *
 *          WHAT THE SCHEDULER IS
 *          ──────────────────────
 *          A lightweight alternative to scattering HAL_GetTick() checks
 *          throughout modes.c.  Without a scheduler, every mode function
 *          would contain ad-hoc timing code:
 *            if (HAL_GetTick() - last_batt  >= 500) { battery_update(); }
 *            if (HAL_GetTick() - last_oled  >= 100) { oled_flush();     }
 *            ...
 *          The scheduler centralises all of this.  Each task is registered
 *          once and polled automatically via scheduler_tick().
 *
 *          DESIGN
 *          ───────
 *          - Fixed table of SCHED_MAX_TASKS task slots.
 *          - Each slot holds: function pointer, interval_ms, last_run_ms.
 *          - scheduler_tick() iterates all slots each call and fires any
 *            task whose (now - last_run) >= interval_ms.
 *          - Tasks execute synchronously inside scheduler_tick() — they
 *            block the loop for their duration.  Keep tasks short (<5 ms).
 *          - scheduler_tick() is called from the main loop in modes.c
 *            inside spin-wait loops and between motion commands.
 *
 *          REGISTERED TASKS AND INTERVALS
 *          ────────────────────────────────
 *          Task                  Interval    Source constant
 *          ──────────────────    ────────    ─────────────────────────
 *          battery_update()      500 ms      BATT_UPDATE_INTERVAL_MS
 *          oled_show_motion()    100 ms      OLED_REFRESH_MS
 *          safety_check()         50 ms      SCHED_SAFETY_INTERVAL_MS
 *          logger_flush()         10 ms      SCHED_LOG_INTERVAL_MS
 *
 *          RELATIONSHIP WITH THE 1 KHz ISR
 *          ─────────────────────────────────
 *          The 1 kHz TIM5 ISR (motion_1khz_tick) runs independently of
 *          the main loop and independently of the scheduler.  The ISR
 *          handles: IR sensors, gyro, encoder reads, PID, motor output.
 *          The scheduler handles: battery polling, display refresh,
 *          safety watchdog, UART log flush.
 *          These two never interfere — the ISR runs between scheduler
 *          task calls.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h   — BATT_UPDATE_INTERVAL_MS, OLED_REFRESH_MS
 *          battery.h  — battery_update(), battery_is_critical()
 *          oled.h     — oled_show_motion(), oled_show_sensors()
 *          safety.h   — safety_check()
 *          logger.h   — LOG_xxx macros
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

/* =========================================================================
 * CONSTANTS
 * ======================================================================= */

/** Maximum number of registered tasks. */
#define SCHED_MAX_TASKS         8U

/** Safety check interval (ms) — battery critical + stall detection. */
#define SCHED_SAFETY_INTERVAL_MS    50U

/** UART log flush interval (ms). */
#define SCHED_LOG_INTERVAL_MS       10U

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  Task function pointer type.
 *
 * @details All scheduler tasks take no arguments and return nothing.
 *          If a task needs to communicate state it uses module-level
 *          functions (e.g. battery_is_critical(), motion_get_status()).
 */
typedef void (*SchedTask_t)(void);

/**
 * @brief  Scheduler task slot.
 */
typedef struct
{
    SchedTask_t  fn;            /**< Task function. NULL = slot unused.    */
    uint32_t     interval_ms;   /**< How often to call fn (ms).            */
    uint32_t     last_run_ms;   /**< HAL_GetTick() when fn last ran.       */
    bool         enabled;       /**< false = task registered but paused.   */
    const char  *name;          /**< Short name for debug output.          */
} SchedSlot_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the scheduler and register the built-in periodic tasks.
 *
 * @details Clears all task slots, then registers:
 *            battery_update()       at BATT_UPDATE_INTERVAL_MS (500 ms)
 *            oled display refresh   at OLED_REFRESH_MS         (100 ms)
 *            safety_check()         at SCHED_SAFETY_INTERVAL_MS (50 ms)
 *            logger_flush()         at SCHED_LOG_INTERVAL_MS    (10 ms)
 *          Call once during system bring-up after all modules are ready.
 *
 * @return MM_OK always.
 */
MmResult_t scheduler_init(void);

/* =========================================================================
 * TASK REGISTRATION
 * ======================================================================= */

/**
 * @brief  Register a periodic task.
 *
 * @details Finds the first empty slot and stores fn, interval_ms, and name.
 *          The task starts enabled and will fire on the next scheduler_tick()
 *          call after interval_ms has elapsed since registration.
 *
 * @param  fn           Task function pointer. Must not be NULL.
 * @param  interval_ms  Repeat interval in milliseconds (> 0).
 * @param  name         Short identifier string for debug output (const).
 *
 * @return MM_OK           Task registered successfully.
 * @return MM_ERR_OVERFLOW All SCHED_MAX_TASKS slots are occupied.
 * @return MM_ERR_PARAM    fn is NULL or interval_ms is 0.
 */
MmResult_t scheduler_register(SchedTask_t fn,
                               uint32_t    interval_ms,
                               const char *name);

/**
 * @brief  Enable or disable a registered task by function pointer.
 *
 * @details Pausing a task (enabled=false) does not remove it — its slot
 *          is retained and it can be re-enabled without re-registering.
 *          The last_run_ms is preserved so re-enabling does not cause
 *          an immediate fire if the interval has not elapsed.
 *
 * @param  fn       Task function pointer to find.
 * @param  enabled  true = run normally, false = skip on each tick.
 *
 * @return MM_OK          Task found and updated.
 * @return MM_ERR_NOT_FOUND  No task with this function pointer registered.
 */
MmResult_t scheduler_set_enabled(SchedTask_t fn, bool enabled);

/* =========================================================================
 * EXECUTION
 * ======================================================================= */

/**
 * @brief  Poll all registered tasks and fire any whose interval has elapsed.
 *
 * @details Call from the main loop as frequently as possible.
 *          Typical call sites in modes.c:
 *            - Inside spin-wait loops during motion (while !done)
 *            - Between successive motion commands
 *            - In the monitor loop (MODE_MONITOR)
 *
 *          Each call checks every slot:
 *            if (enabled && (now - last_run) >= interval_ms):
 *              last_run = now
 *              fn()
 *
 *          Tasks are called in registration order.  If a task takes a
 *          long time (e.g. oled_flush ~2.6 ms), subsequent tasks in the
 *          same tick may be delayed.  This is acceptable — none of the
 *          registered tasks are hard-real-time.
 *
 * @note   Do NOT call from the 1 kHz ISR.  Main loop only.
 */
void scheduler_tick(void);

/**
 * @brief  Run a single named task immediately, bypassing the interval.
 *
 * @details Useful for forcing an immediate battery check after a run,
 *          or refreshing the OLED display right after mode change.
 *          Does NOT update last_run_ms — the next scheduled fire is
 *          unaffected.
 *
 * @param  fn  Task function pointer to fire immediately.
 *
 * @return MM_OK           Task found and executed.
 * @return MM_ERR_NOT_FOUND  Task not registered.
 */
MmResult_t scheduler_run_now(SchedTask_t fn);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Return the number of currently registered tasks.
 *
 * @return uint8_t  Count of non-NULL slots [0, SCHED_MAX_TASKS].
 */
uint8_t scheduler_task_count(void);

/**
 * @brief  Print all registered tasks and their intervals via UART logger.
 *
 * @details Output format per task:
 *            "[0] battery_update    500 ms  ENABLED"
 *            "[1] oled_refresh      100 ms  ENABLED"
 *            ...
 */
void scheduler_print_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
