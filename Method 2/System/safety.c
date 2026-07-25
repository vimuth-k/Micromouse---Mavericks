/**
 * @file    safety.c
 * @brief   MicroMaze 3 · Safety watchdog — implementation.
 * @details See safety.h for the full design rationale behind each of the
 *          five checks and the run-lifecycle / trip-latch model.
 *
 * @author  VDawn
 * @date    2026
 */
#include "safety.h"
#include "config.h"
#include "error.h"
#include "logger.h"
#include "battery.h"
#include "motors.h"
#include "motion.h"
#include "main.h"     /* HAL_GetTick() */

/* ═══════════════════════════════════════════════════════════════════════
 * Module state
 * ═══════════════════════════════════════════════════════════════════════ */

/** HAL_GetTick() at the last safety_run_start(), 0 if no run active. */
static uint32_t s_run_start_ms = 0U;

/** Cumulative odometry since the last safety_run_start() (mm). */
static float s_cumulative_dist_mm = 0.0f;

/** motion_get_status().position_mm as of the previous safety_check()
 *  call — used to detect per-move resets when accumulating distance. */
static float s_last_position_mm = 0.0f;

/** HAL_GetTick() when the stall condition first became true, 0 if the
 *  robot is not currently below the stall speed threshold. */
static uint32_t s_stall_since_ms = 0U;

/** Latched trip state — persists until safety_clear(). */
static bool s_tripped = false;

/** Which check most recently latched a trip. */
static SafetyTripReason_t s_trip_reason = SAFETY_TRIP_NONE;

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Latch a trip: record the reason, log it once, and stop the
 *         robot. Safe to call repeatedly — logging and the stop
 *         commands are idempotent from the caller's side (motion_stop()
 *         and motors_disable() are both safe to call when already
 *         stopped/disabled).
 */
static void trip(SafetyTripReason_t reason, const char *why)
{
    /* Only log on the transition into a trip (or a change of reason)
     * to avoid spamming the UART at 20 Hz while a condition persists. */
    if (!s_tripped || (s_trip_reason != reason))
    {
        LOG_ERROR("SAFETY TRIP: %s", why);
    }

    s_tripped     = true;
    s_trip_reason = reason;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t safety_init(void)
{
    s_run_start_ms        = 0U;
    s_cumulative_dist_mm  = 0.0f;
    s_last_position_mm    = 0.0f;
    s_stall_since_ms      = 0U;
    s_tripped             = false;
    s_trip_reason         = SAFETY_TRIP_NONE;

    return MM_OK;
}

void safety_run_start(void)
{
    s_run_start_ms       = HAL_GetTick();
    s_cumulative_dist_mm = 0.0f;
    s_last_position_mm   = 0.0f;
    s_stall_since_ms     = 0U;
}

void safety_check(void)
{
    /* ── 1. Battery critical — second line of defence at 20 Hz ───────── */
    if (battery_is_critical())
    {
        motors_disable();
        trip(SAFETY_TRIP_BATTERY_CRITICAL, "battery critical");
        /* Keep evaluating the other checks below — a tripped battery
         * doesn't make the remaining checks meaningless, and cheap to
         * run regardless. */
    }

    MotionStatus_t status;
    motion_get_status(&status);

    /* ── 2. Motor stall (PWM-qualified) ───────────────────────────────── */
    MotorState_t mstate;
    motors_get_state(&mstate);

    int32_t pwm_l = (mstate.pwm_left  < 0) ? -mstate.pwm_left  : mstate.pwm_left;
    int32_t pwm_r = (mstate.pwm_right < 0) ? -mstate.pwm_right : mstate.pwm_right;
    bool driven = mstate.enabled &&
                  ((pwm_l > (int32_t)SAFETY_STALL_PWM_THRESH) ||
                   (pwm_r > (int32_t)SAFETY_STALL_PWM_THRESH));

    if (driven)
    {
        float avg_spd = (status.meas_speed_l + status.meas_speed_r) * 0.5f;
        if (avg_spd < 0.0f) { avg_spd = -avg_spd; }

        if (avg_spd < SAFETY_STALL_SPD_MMPS)
        {
            if (s_stall_since_ms == 0U)
            {
                s_stall_since_ms = HAL_GetTick();
            }
            else if ((HAL_GetTick() - s_stall_since_ms) >= SAFETY_STALL_TIME_MS)
            {
                motion_stop();
                trip(SAFETY_TRIP_STALL, "motor stall detected");
            }
        }
        else
        {
            s_stall_since_ms = 0U; /* Moving normally — reset the timer. */
        }
    }
    else
    {
        s_stall_since_ms = 0U; /* Not driven hard enough to qualify. */
    }

    /* ── 3 & 4. Run timeout and max distance (only while a run is active) */
    if (s_run_start_ms != 0U)
    {
        if ((HAL_GetTick() - s_run_start_ms) >= SAFETY_MAX_RUN_MS)
        {
            motion_stop();
            trip(SAFETY_TRIP_RUN_TIMEOUT, "run time limit exceeded");
        }

        /* Accumulate odometry. motion.c's position_mm resets to ~0 at
         * the start of each new move, so a negative delta means a new
         * move began since the last check — treat position_mm as the
         * new baseline rather than subtracting a bogus negative. */
        float delta = status.position_mm - s_last_position_mm;
        if (delta > 0.0f)
        {
            s_cumulative_dist_mm += delta;
        }
        s_last_position_mm = status.position_mm;

        if (s_cumulative_dist_mm >= SAFETY_MAX_DIST_MM)
        {
            motion_stop();
            trip(SAFETY_TRIP_MAX_DISTANCE, "max run distance exceeded");
        }
    }

    /* ── 5. Yaw error while driving straight ──────────────────────────── */
    if (status.state == MOTION_FORWARD)
    {
        float yaw_err = status.yaw_deg;
        if (yaw_err < 0.0f) { yaw_err = -yaw_err; }

        if (yaw_err > SAFETY_MAX_YAW_DEG)
        {
            motion_stop();
            trip(SAFETY_TRIP_YAW_ERROR, "yaw error exceeded limit while driving straight");
        }
    }
}

bool safety_is_tripped(void)
{
    return s_tripped;
}

SafetyTripReason_t safety_trip_reason(void)
{
    return s_trip_reason;
}

void safety_clear(void)
{
    s_tripped     = false;
    s_trip_reason = SAFETY_TRIP_NONE;
}
