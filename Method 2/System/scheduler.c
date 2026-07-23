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
 *            Handles: IR sensors, gyro, encoders, PID, motor output.
 *            Latency requirement: < 1 ms.
 *            Must never block.
 *
 *          Context 2 — Main loop (scheduler_tick inside modes.c)
 *            Handles: battery poll, OLED refresh, safety watchdog, LED.
 *            Latency requirement: best-effort, ≤ 10 ms acceptable.
 *            May block briefly (OLED I2C ~2.6 ms, logger UART ~0.5 ms).
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
 *          slot 3  log_flush_task          10 ms   drains UART tx buffer
 *          slot 4  led_heartbeat_task     500 ms   toggles onboard LED
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
#include "pins.h"
#include "battery.h"
#include "oled.h"
#include "motors.h"
#include "motion.h"
#include "logger.h"
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
 *          Logs a warning the first time voltage drops into LOW state.
 *          Logs a critical error if voltage drops to CRITICAL state.
 */
static void battery_update_task(void)
{
    battery_update();

    if (battery_is_critical())
    {
        LOG_ERROR("BATTERY CRITICAL: %.2f V — disabling motors",
                  (double)battery_voltage());
        motors_disable();
    }
    else if (battery_is_low())
    {
        LOG_WARN("Battery low: %.2f V (%u%%)",
                 (double)battery_voltage(), battery_percent());
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
 * @details Checks:
 *          1. Battery critical → disable motors (also done in battery task,
 *             this is a second line of defence at higher frequency).
 *          2. Motor stall detection: if motors are enabled, a speed
 *             target is set, but measured speed < SAFETY_STALL_SPD_MMPS
 *             for SAFETY_STALL_TIME_MS → declare stall and stop.
 *
 *          This runs at 50 ms (20 Hz) — fast enough to catch a stall
 *          before the robot damages itself against a wall.
 */
static void safety_check_task(void)
{
    /* Battery critical — hard stop */
    if (battery_is_critical())
    {
        motors_disable();
        return;
    }

    /* Stall detection only when motors are active and robot should be moving */
    if (!motors_is_enabled()) { return; }
    if (motion_is_idle())     { return; }

    static uint32_t s_stall_since = 0U;

    float spd_l = motion_speed_left_mmps();
    float spd_r = motion_speed_right_mmps();
    float avg_spd = (spd_l + spd_r) * 0.5f;
    if (avg_spd < 0.0f) { avg_spd = -avg_spd; }

    if (avg_spd < SAFETY_STALL_SPD_MMPS)
    {
        if (s_stall_since == 0U)
        {
            s_stall_since = HAL_GetTick();
        }
        else if ((HAL_GetTick() - s_stall_since) >= SAFETY_STALL_TIME_MS)
        {
            LOG_ERROR("Motor stall detected — stopping");
            motion_stop();
            s_stall_since = 0U;
        }
    }
    else
    {
        s_stall_since = 0U;   /* Reset stall timer when moving normally */
    }
}

/**
 * @brief  Logger flush task (10 ms).
 *
 * @details The logger buffers outgoing UART messages to avoid blocking
 *          the main loop.  This task flushes any pending bytes.
 *          At 115200 baud, 10 ms allows up to 115 characters to transmit.
 */
static void log_flush_task(void)
{
    logger_flush();
}

/**
 * @brief  LED heartbeat task (500 ms).
 *
 * @details Toggles the onboard LED at 1 Hz (on 500 ms, off 500 ms)
 *          to indicate the main loop is alive and the scheduler is running.
 *          If the LED stops blinking during a run, the main loop has
 *          stalled (scheduler_tick() is not being called).
 *          During a run the motion ISR keeps motors moving even if the
 *          main loop stalls — but the scheduler would stop and the display
 *          would freeze.  The heartbeat makes this visible immediately.
 */
static void led_heartbeat_task(void)
{
    LED_TOGGLE();
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

    r = scheduler_register(log_flush_task, SCHED_LOG_INTERVAL_MS,
                           "log_flush");
    if (r != MM_OK) { return r; }

    r = scheduler_register(led_heartbeat_task, SCHED_LED_HEARTBEAT_MS,
                           "led_heartbeat");
    if (r != MM_OK) { return r; }

    LOG_DEBUG("Scheduler: %u tasks registered", scheduler_task_count());
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
 * @brief  Print all registered tasks via UART logger.
 */
void scheduler_print_tasks(void)
{
    LOG_INFO("Scheduler tasks (%u/%u slots used):",
             scheduler_task_count(), SCHED_MAX_TASKS);

    for (uint8_t i = 0U; i < SCHED_MAX_TASKS; i++)
    {
        const SchedSlot_t *slot = &s_slots[i];
        if (slot->fn == NULL) { continue; }

        LOG_INFO("  [%u] %-20s  %4lu ms  %s",
                 i,
                 slot->name,
                 (unsigned long)slot->interval_ms,
                 slot->enabled ? "ENABLED" : "PAUSED");
    }
}
