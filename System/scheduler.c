/**
 * @file    scheduler.c
 * @brief   Cooperative task scheduler — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          Manages a fixed table of periodic tasks that run from the main
 *          loop.  Every call to scheduler_tick() scans the table and fires
 *          any task whose interval has elapsed since it last ran.
 *
 *          THE TWO EXECUTION CONTEXTS IN THIS FIRMWARE
 *          ─────────────────────────────────────────────
 *          Context 1 — TIM5 ISR at 1 kHz (motion_1khz_tick)
 *           *          Context 2 — Main loop (scheduler_tick inside modes.c)
 *            Handles: battery poll, OLED refresh, safety watchdog.
 *            Latency requirement: best-effort, ≤ 10 ms acceptable.
 *            May block briefly (OLED I2C ~2.6 ms).
 *
 *          The scheduler owns Context 2.  The ISR owns Context 1.
 *          They never share data directly — each module that needs
 *          cross-context data uses volatile variables or cached readings.
 *
 *          BUILT-IN TASK TABLE
 *          ────────────────────
 *          slot 0  battery_update_task    500 ms   polls shared ADC slot
 *          slot 1  oled_refresh_task      100 ms   flushes framebuffer
 *          slot 2  safety_check_task       50 ms   battery + stall check
 *
 *          HOW scheduler_tick() IS CALLED
 *          ────────────────────────────────
 *          modes.c calls scheduler_tick() at two points:
 *          a) Inside the spin-wait during motion:
 *               while (!motion_is_idle()) { scheduler_tick(); }
 *          b) In the monitor loop:
 *               while (1) { sensors_update(); scheduler_tick(); }
 *          At SPD_SEARCH (200 mm/s) a 1-cell move takes ~0.9 s.
 *          The scheduler fires battery (once) and OLED (9×) during that move.
 *
 *          TIMER ARITHMETIC AND ROLLOVER
 *          ──────────────────────────────
 *          HAL_GetTick() returns a uint32_t that rolls over after
 *          2^32 ms ≈ 49.7 days.  The check:
 *            (now - last_run) >= interval
 *          uses unsigned subtraction which handles rollover correctly
 *          as long as interval_ms < 2^31 ms (~24.8 days).
 *          All intervals here are ≤ 500 ms, so rollover is never an issue.
 *
 * @author  VDawn
 * @date    2026
 */

#include "scheduler.h"
#include "config.h"
#include "battery.h"
#include "oled.h"
#include "motors.h"
#include "motion.h"
#include "safety.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* =========================================================================
 * PRIVATE — TASK TABLE
 * ======================================================================= */

/** All registered task slots. */
static SchedSlot_t s_slots[SCHED_MAX_TASKS];

/* =========================================================================
 * PRIVATE — BUILT-IN TASK IMPLEMENTATIONS
 * ======================================================================= */

/**
 * @brief  Battery polling task (500 ms).
 *
 * @details Calls battery_update() which reads the shared ADC DMA slot
 *          from ir.c, applies the IIR filter, and updates the health state.
 *          Disables motors if voltage drops to CRITICAL state.
 */
static void battery_update_task(void)
{
    battery_update();

    if (battery_is_critical())
    {
        motors_disable();
    }
}

/**
 * @brief  OLED display refresh task (100 ms).
 *
 * @details Refreshes the display with current motion status when a run
 *          is in progress, or sensor values when idle.
 *          The content selection is intentionally simple — oled.c has
 *          the full set of display pages that modes.c can use explicitly.
 */
static void oled_refresh_task(void)
{
#if OLED_ENABLED
    if (!motion_is_idle())
    {
        /* During a run — show motion data */
        MotionStatus_t status;
        motion_get_status(&status);
        oled_show_motion(
            status.meas_speed_l,
            status.meas_speed_r,
            status.position_mm,
            status.yaw_deg,
            status.tracking_err_mm,
            maze.robot_row,
            maze.robot_col,
            maze.robot_heading,
            HAL_GetTick(),
            battery_voltage()
        );
    }
    /* When idle: modes.c updates the display explicitly */
#endif
}

/**
 * @brief  Safety watchdog task (50 ms).
 *
 * @details Thin wrapper around safety_check() (System/safety.c), which
 *          owns all five checks — battery critical, PWM-qualified motor
 *          stall, run timeout, max distance, and yaw error. Kept as a
 *          named wrapper (rather than registering safety_check directly)
 *          purely for naming consistency with the other *_task functions
 *          in this table.
 */
static void safety_check_task(void)
{
    safety_check();
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the scheduler and register built-in tasks.
 */
MmResult_t scheduler_init(void)
{
    (void)memset(s_slots, 0, sizeof(s_slots));

    MmResult_t r;

    r = scheduler_register(battery_update_task, BATT_UPDATE_INTERVAL_MS,
                           "battery_update");
    if (r != MM_OK) { return r; }

    r = scheduler_register(oled_refresh_task, OLED_REFRESH_MS,
                           "oled_refresh");
    if (r != MM_OK) { return r; }

    r = scheduler_register(safety_check_task, SCHED_SAFETY_INTERVAL_MS,
                           "safety_check");
    if (r != MM_OK) { return r; }

    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — TASK REGISTRATION
 * ======================================================================= */

/**
 * @brief  Register a periodic task in the first available slot.
 */
MmResult_t scheduler_register(SchedTask_t fn,
                               uint32_t    interval_ms,
                               const char *name)
{
    if (fn == NULL || interval_ms == 0U) { return MM_ERR_PARAM; }

    for (uint8_t i = 0U; i < SCHED_MAX_TASKS; i++)
    {
        if (s_slots[i].fn == NULL)
        {
            s_slots[i].fn          = fn;
            s_slots[i].interval_ms = interval_ms;
            s_slots[i].last_run_ms = HAL_GetTick();
            s_slots[i].enabled     = true;
            s_slots[i].name        = (name != NULL) ? name : "unnamed";
            return MM_OK;
        }
    }

    return MM_ERR_OVERFLOW;
}

/**
 * @brief  Enable or disable a registered task by function pointer.
 */
MmResult_t scheduler_set_enabled(SchedTask_t fn, bool enabled)
{
    if (fn == NULL) { return MM_ERR_PARAM; }

    for (uint8_t i = 0U; i < SCHED_MAX_TASKS; i++)
    {
        if (s_slots[i].fn == fn)
        {
            s_slots[i].enabled = enabled;
            return MM_OK;
        }
    }

    return MM_ERR_NOT_FOUND;
}

/* =========================================================================
 * PUBLIC API — EXECUTION
 * ======================================================================= */

/**
 * @brief  Poll all registered tasks and fire any whose interval has elapsed.
 *
 * @details Core polling loop:
 *            for each slot:
 *              if slot.fn != NULL and slot.enabled:
 *                now = HAL_GetTick()
 *                if (now - slot.last_run_ms) >= slot.interval_ms:
 *                  slot.last_run_ms = now
 *                  slot.fn()
 *
 *          HAL_GetTick() is read per-slot (not once at the top) so that
 *          if an earlier task takes >1 ms, a later task in the same tick
 *          can still fire immediately if its interval has also elapsed.
 *          This trades one extra tick read per slot for more accurate
 *          inter-task timing when tasks occasionally run long.
 */
void scheduler_tick(void)
{
    for (uint8_t i = 0U; i < SCHED_MAX_TASKS; i++)
    {
        SchedSlot_t *slot = &s_slots[i];

        if (slot->fn == NULL)     { continue; }
        if (!slot->enabled)       { continue; }

        uint32_t now = HAL_GetTick();

        if ((now - slot->last_run_ms) >= slot->interval_ms)
        {
            slot->last_run_ms = now;
            slot->fn();
        }
    }
}

/**
 * @brief  Run a single task immediately, bypassing the interval.
 */
MmResult_t scheduler_run_now(SchedTask_t fn)
{
    if (fn == NULL) { return MM_ERR_PARAM; }

    for (uint8_t i = 0U; i < SCHED_MAX_TASKS; i++)
    {
        if (s_slots[i].fn == fn)
        {
            s_slots[i].fn();
            return MM_OK;
        }
    }

    return MM_ERR_NOT_FOUND;
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Return count of registered (non-NULL) task slots.
 */
uint8_t scheduler_task_count(void)
{
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < SCHED_MAX_TASKS; i++)
    {
        if (s_slots[i].fn != NULL) { count++; }
    }
    return count;
}

/**
 * @brief  Print all registered tasks (stub when logging disabled).
 */
void scheduler_print_tasks(void)
{
    /* No-op: logging is disabled */
}
