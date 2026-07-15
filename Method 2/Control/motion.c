/**
 * @file    motion.c
 * @brief   Motion control — implementation.
 *
 * @details ARCHITECTURE OVERVIEW
 *          ─────────────────────
 *          motion.c runs one function from a hardware timer interrupt
 *          every millisecond: motion_1khz_tick().  Everything else in
 *          this file exists to support that one function.
 *
 *          The tick runs a layered correction pipeline:
 *
 *          ┌─────────────────────────────────────────────────────────┐
 *          │  motion_1khz_tick()  (runs every 1 ms from TIM5 ISR)   │
 *          │                                                          │
 *          │  1. ir_update()          — fire/read all 6 IR pairs     │
 *          │  2. imu_update()         — integrate gyro yaw           │
 *          │  3. enc_left/right_delta — encoder Δ counts this tick   │
 *          │  4. velocity profile     — compute ramp speed this tick │
 *          │  5. FSM dispatch:                                        │
 *          │       FORWARD  → layers 1–4 (below)                     │
 *          │       TURNING  → turn PID                               │
 *          │       ALIGNING → alignment nudge                        │
 *          │       BRAKING  → motors_brake() + settle                │
 *          │       IDLE     → motors_stop()                          │
 *          │                                                          │
 *          │  LAYER 1: Speed PID (per wheel)                         │
 *          │    error  = ramp_speed − meas_speed_X                   │
 *          │    output = base PWM for each wheel                      │
 *          │                                                          │
 *          │  LAYER 2: Straightness PID (encoder difference)         │
 *          │    error  = enc_left_count − enc_right_count            │
 *          │    output = ±correction  (−left, +right)                │
 *          │                                                          │
 *          │  LAYER 3: Heading PID (gyro yaw)                        │
 *          │    error  = 0° − imu_yaw_deg()                          │
 *          │    output = ±correction  (−left, +right)                │
 *          │                                                          │
 *          │  LAYER 4: Wall centering (IR side sensors)              │
 *          │    error  = ir_side_error()                             │
 *          │    output = ±correction  (−left, +right)                │
 *          │    active = only when both side walls present            │
 *          │                                                          │
 *          │  Final PWM:                                              │
 *          │    left  = speed_L − straight − heading − wall          │
 *          │    right = speed_R + straight + heading + wall          │
 *          │    → motors_set(left, right)                            │
 *          └─────────────────────────────────────────────────────────┘
 *
 *          TRAPEZOIDAL VELOCITY PROFILE
 *          ─────────────────────────────
 *          The ramp speed is computed each tick as:
 *
 *          Acceleration zone:
 *            if ramp_speed < target_speed:
 *              ramp_speed += ACCEL × 0.001   (mm/s per tick)
 *
 *          Braking zone (triggered when remaining ≤ brake_dist):
 *            brake_dist = ramp_speed² / (2 × DECEL)
 *            ramp_speed -= DECEL × 0.001
 *
 *          The speed PID tracks ramp_speed (a smooth ramp) rather than
 *          target_speed (a step).  This prevents the PID from seeing
 *          an impossible step demand on move start, which would wind up
 *          the integral and cause overshoot.
 *
 *          BLOCKING + VOLATILE FLAG PROTOCOL
 *          ───────────────────────────────────
 *          Blocking functions (move_forward, turns, align) set
 *          s_motion_done = false, start the FSM, then spin:
 *            while (!s_motion_done) {}
 *          The ISR sets s_motion_done = true when complete.
 *          s_motion_done is volatile — the compiler must not cache it.
 *          The main loop has nothing else to do during a move, so the
 *          busy-wait is correct here.  If you add a scheduler later,
 *          replace the spin with a semaphore or event flag.
 *
 * @author  VDawn
 * @date    2026
 */

#include "motion.h"
#include "config.h"
#include "pins.h"
#include "pid.h"
#include "motors.h"
#include "encoders.h"
#include "ir.h"
#include "imu.h"
#include "utils.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/* =========================================================================
 * PRIVATE — PID INSTANCES
 * ======================================================================= */

/** Left  wheel speed controller. */
static PID_t s_pid_spd_l;

/** Right wheel speed controller. */
static PID_t s_pid_spd_r;

/**
 * @brief  Straightness controller — encoder count difference.
 *
 * @details Error = enc_left_count − enc_right_count (counts).
 *          Positive error = left ahead = steer right (add to right, sub from left).
 */
static PID_t s_pid_straight;

/**
 * @brief  Heading controller — gyro yaw correction during straights.
 *
 * @details Error = 0° − imu_yaw_deg() (target heading is always 0
 *          because gyro is reset at the start of every move).
 *          Positive error = robot drifted left = correct right.
 */
static PID_t s_pid_heading;

/**
 * @brief  Turn controller — gyro yaw error during pivot turns.
 *
 * @details Error = turn_target_deg − imu_yaw_deg().
 *          Positive error = need to turn more clockwise.
 */
static PID_t s_pid_turn;

/* =========================================================================
 * PRIVATE — FSM STATE
 * ======================================================================= */

/** Current FSM state — written by blocking functions, read by ISR. */
static volatile MotionState_t s_state = MOTION_IDLE;

/**
 * @brief  Completion flag — ISR sets true, blocking function clears false.
 * @note   Volatile: compiler must re-read this every loop iteration.
 */
static volatile bool s_motion_done = false;

/* =========================================================================
 * PRIVATE — PROFILE STATE (written & read by ISR)
 * ======================================================================= */

/** Target cruise speed for the current move (mm/s). */
static volatile float s_target_speed = 0.0f;

/** Current ramp profile speed (mm/s) — tracks toward s_target_speed. */
static volatile float s_ramp_speed = 0.0f;

/** Total distance to travel for current forward move (mm). */
static volatile float s_target_dist_mm = 0.0f;

/** Target heading for pivot turn (degrees, +CW, −CCW). */
static volatile float s_turn_target_deg = 0.0f;

/** Acceleration to use for current move (mm/s²). */
static volatile float s_accel = ACCEL_NORMAL;

/** Deceleration to use for current move (mm/s²). */
static volatile float s_decel = DECEL_NORMAL;

/* =========================================================================
 * PRIVATE — MEASUREMENTS (updated by ISR each tick, read by main)
 * ======================================================================= */

static volatile float s_meas_spd_l = 0.0f;   /**< Left  wheel speed mm/s  */
static volatile float s_meas_spd_r = 0.0f;   /**< Right wheel speed mm/s  */

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Reset all five PID instances and encoder/gyro references.
 *
 * @details Called before every move so stale integral state from the
 *          previous manoeuvre does not affect the new one.
 */
static void reset_all_controllers(void)
{
    pid_reset(&s_pid_spd_l);
    pid_reset(&s_pid_spd_r);
    pid_reset(&s_pid_straight);
    pid_reset(&s_pid_heading);
    pid_reset(&s_pid_turn);
}

/**
 * @brief  Clamp a float to [lo, hi].
 */
static float clamp_f(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief  Compute the ramp profile speed for one tick.
 *
 * @details Implements trapezoidal velocity:
 *          - Acceleration zone  : ramp_speed ramps toward target at ACCEL.
 *          - Cruising zone      : ramp_speed == target_speed.
 *          - Braking zone       : when remaining_dist ≤ braking_distance,
 *                                 ramp_speed decelerates at DECEL.
 *          - Creep floor        : never drops below SPD_CREEP until
 *                                 remaining_dist < 2 mm (final stop).
 *
 * @param  remaining_mm  Distance remaining to target (mm).
 * @return float         Ramp speed for this tick (mm/s).
 */
static float compute_ramp_speed(float remaining_mm)
{
    /* Distance needed to decelerate from current ramp speed to 0 */
    float brake_dist = (s_ramp_speed * s_ramp_speed) / (2.0f * s_decel);

    if (remaining_mm <= 2.0f)
    {
        /* Final stop — drop to zero */
        return 0.0f;
    }
    else if (remaining_mm <= brake_dist)
    {
        /* Braking zone — decelerate */
        float new_spd = s_ramp_speed - (s_decel * CTRL_LOOP_DT);
        return clamp_f(new_spd, SPD_CREEP, s_ramp_speed);
    }
    else if (s_ramp_speed < s_target_speed)
    {
        /* Acceleration zone */
        float new_spd = s_ramp_speed + (s_accel * CTRL_LOOP_DT);
        return clamp_f(new_spd, 0.0f, s_target_speed);
    }
    else
    {
        /* Cruise zone */
        return s_target_speed;
    }
}

/* =========================================================================
 * PRIVATE — ISR HANDLERS (one per FSM state)
 * All run inside motion_1khz_tick() — must complete in < 250 µs total.
 * ======================================================================= */

/**
 * @brief  FORWARD state handler — layered PID control.
 *
 * @details Called every tick while MOTION_FORWARD is active.
 *          Reads fresh sensor/encoder/gyro data (already updated by
 *          the caller before dispatch) and applies the 4-layer correction.
 */
static void handle_forward(void)
{
    /* ── Distance check ──────────────────────────────────────────── */
    float pos_mm      = enc_avg_mm();
    float remaining   = s_target_dist_mm - pos_mm;

    /* Update ramp speed */
    s_ramp_speed = compute_ramp_speed(remaining);

    if (s_ramp_speed <= 0.0f && remaining <= 2.0f)
    {
        /* Move complete — transition to braking */
        s_state = MOTION_BRAKING;
        return;
    }

    /* ── LAYER 1: Speed PID (per wheel) ─────────────────────────── */
    /* Convert encoder deltas (counts/tick) to mm/s                  */
    float spd_l = enc_left_speed_mmps();
    float spd_r = enc_right_speed_mmps();
    s_meas_spd_l = spd_l;
    s_meas_spd_r = spd_r;

    float pwm_l = pid_update(&s_pid_spd_l, s_ramp_speed - spd_l);
    float pwm_r = pid_update(&s_pid_spd_r, s_ramp_speed - spd_r);

    /* ── LAYER 2: Straightness PID (encoder difference) ─────────── */
    float enc_diff  = (float)(enc_left_count() - enc_right_count());
    float straight  = pid_update(&s_pid_straight, enc_diff);

    /* ── LAYER 3: Heading PID (gyro yaw) ────────────────────────── */
    float heading   = pid_update(&s_pid_heading, -imu_yaw_deg());

    /* ── LAYER 4: Wall centering (IR side sensors) ──────────────── */
    float wall = 0.0f;
    if (ir_wall_left() && ir_wall_right())
    {
        wall = KP_WALL_CENTER * ir_side_error();
    }

    /* ── Combine all corrections ─────────────────────────────────── */
    /*   Positive straight/heading/wall → robot drifted left         */
    /*   → subtract from left, add to right to steer right           */
    float final_l = pwm_l - straight - heading - wall;
    float final_r = pwm_r + straight + heading + wall;

    /* Clamp to valid PWM range — motors_set() also clamps, but      */
    /* clamping here prevents large corrections from inverting speed  */
    final_l = clamp_f(final_l, 0.0f, (float)PWM_MAX);
    final_r = clamp_f(final_r, 0.0f, (float)PWM_MAX);

    motors_set((int32_t)final_l, (int32_t)final_r);
}

/**
 * @brief  TURNING state handler — gyro-based pivot turn.
 *
 * @details Uses a single PID on the heading error.  Both motors run at
 *          equal and opposite speeds for a true pivot.  Minimum PWM
 *          (TURN_PWM_MIN) prevents stall at small residual errors.
 */
static void handle_turning(void)
{
    float yaw   = imu_yaw_deg();
    float error = s_turn_target_deg - yaw;

    /* Check completion */
    float abs_error = (error < 0.0f) ? -error : error;
    if (abs_error < TURN_TOLERANCE_DEG)
    {
        s_state = MOTION_BRAKING;
        return;
    }

    /* Turn PID — output is the magnitude of the pivot PWM */
    float pwm = pid_update(&s_pid_turn, abs_error);

    /* Apply floor */
    if (pwm < TURN_PWM_MIN) { pwm = TURN_PWM_MIN; }

    /* Direction: positive error = need to turn clockwise (right)    */
    /* Clockwise pivot: left fwd, right bwd                          */
    if (error > 0.0f)
    {
        motors_set( (int32_t)pwm, -(int32_t)pwm);
    }
    else
    {
        motors_set(-(int32_t)pwm,  (int32_t)pwm);
    }
}

/**
 * @brief  ALIGNING state handler — square up to front wall using IR.
 *
 * @details Drives slowly forward while applying a yaw correction from
 *          the front sensor imbalance (ir_front_error).  Completes
 *          when front wall is close AND sensors are balanced.
 */
static void handle_aligning(void)
{
    if (!ir_wall_front())
    {
        /* Front wall disappeared — abort alignment */
        s_state = MOTION_BRAKING;
        return;
    }

    float ferr = ir_front_error();   /* LF − RF: positive = nose right */

    /* Slow forward nudge with yaw correction                         */
    float corr  = KP_ALIGN * ferr;
    int32_t base = 400;

    int32_t pwm_l = (int32_t)clamp_f((float)base - corr, 0.0f, 800.0f);
    int32_t pwm_r = (int32_t)clamp_f((float)base + corr, 0.0f, 800.0f);
    motors_set(pwm_l, pwm_r);

    /* Complete when close AND balanced */
    float abs_ferr = (ferr < 0.0f) ? -ferr : ferr;
    if (ir_wall_front_close() && abs_ferr < 40.0f)
    {
        s_state = MOTION_BRAKING;
    }
}

/**
 * @brief  BRAKING state handler — one-tick active brake then IDLE.
 *
 * @details Applies motors_brake() immediately.  The next tick the FSM
 *          transitions to IDLE and the blocking caller is unblocked.
 *          A HAL_Delay is NOT used inside the ISR — instead the brake
 *          is held for one tick only.  The blocking function adds the
 *          settle delay after s_motion_done is set.
 */
static void handle_braking(void)
{
    motors_brake();
    s_state      = MOTION_IDLE;
    s_motion_done = true;  /* Unblocks the waiting main-loop function  */
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the motion control module.
 */
MmResult_t motion_init(void)
{
    /* Speed PIDs — 4-tap D filter to smooth encoder quantisation noise */
    pid_init_ex(&s_pid_spd_l,
                KP_SPEED, KI_SPEED, KD_SPEED,
                SPEED_INTEG_LIMIT, (float)PWM_MAX, 4U);

    pid_init_ex(&s_pid_spd_r,
                KP_SPEED, KI_SPEED, KD_SPEED,
                SPEED_INTEG_LIMIT, (float)PWM_MAX, 4U);

    /* Straightness PID — no D filter (count differences are smoother) */
    pid_init(&s_pid_straight,
             KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT,
             STRAIGHT_INTEG_LIMIT, STRAIGHT_OUTPUT_LIMIT);

    /* Heading PID */
    pid_init(&s_pid_heading,
             KP_HEADING, KI_HEADING, KD_HEADING,
             HEADING_INTEG_LIMIT, HEADING_OUTPUT_LIMIT);

    /* Turn PID — no I or D filter (gyro is already smooth) */
    pid_init(&s_pid_turn,
             KP_TURN, KI_TURN, KD_TURN,
             TURN_INTEG_LIMIT, TURN_PWM_MAX);

    s_state       = MOTION_IDLE;
    s_motion_done = false;
    s_ramp_speed  = 0.0f;

    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — BLOCKING MOVE PRIMITIVES
 * ======================================================================= */

/**
 * @brief  Drive forward N cells using a trapezoidal velocity profile.
 */
void move_forward(uint8_t cells, float speed_mmps)
{
    /* Prepare */
    encoders_reset();
    imu_reset_yaw();
    reset_all_controllers();
    motors_enable();

    /* Choose accel/decel profile based on speed */
    if (speed_mmps <= SPD_SEARCH)
    {
        s_accel = ACCEL_SEARCH;
        s_decel = DECEL_SEARCH;
    }
    else
    {
        s_accel = ACCEL_NORMAL;
        s_decel = DECEL_NORMAL;
    }

    /* Configure move parameters */
    s_target_dist_mm = (float)cells * CELL_WIDTH_MM;
    s_target_speed   = clamp_f(speed_mmps, SPD_CREEP, SPD_ABSOLUTE_MAX);
    s_ramp_speed     = 0.0f;
    s_motion_done    = false;
    s_state          = MOTION_FORWARD;   /* ISR begins executing       */

    /* Block until ISR sets s_motion_done */
    while (!s_motion_done) {}

    /* Settle — let active brake damp oscillation */
    HAL_Delay(TURN_SETTLE_MS);
    motors_coast();
}

/**
 * @brief  Shared pivot turn implementation.
 *
 * @param  angle_deg  Target heading (positive = CW / right,
 *                                    negative = CCW / left).
 */
static void do_turn(float angle_deg)
{
    imu_reset_yaw();
    reset_all_controllers();
    motors_enable();

    s_turn_target_deg = angle_deg;
    s_motion_done     = false;
    s_state           = MOTION_TURNING;

    while (!s_motion_done) {}

    HAL_Delay(TURN_SETTLE_MS);
    motors_coast();

    /* Reset yaw so the next straight move starts with reference = 0° */
    imu_reset_yaw();
}

/**
 * @brief  Pivot 90° clockwise (right).
 */
void motion_turn_right(void)
{
    do_turn(+90.0f);
}

/**
 * @brief  Pivot 90° counter-clockwise (left).
 */
void motion_turn_left(void)
{
    do_turn(-90.0f);
}

/**
 * @brief  Pivot 180° U-turn (clockwise).
 */
void motion_turn_180(void)
{
    do_turn(+180.0f);
}

/**
 * @brief  Align to front wall using IR balance.
 */
void motion_align_front(void)
{
    if (!ir_wall_front())
    {
        return;   /* No front wall — alignment is a no-op */
    }

    reset_all_controllers();
    motors_enable();

    s_motion_done = false;
    s_state       = MOTION_ALIGNING;

    /* Timeout: 1 second max — prevents infinite nudge if wall disappears */
    uint32_t deadline = HAL_GetTick() + 1000U;
    while (!s_motion_done && (HAL_GetTick() < deadline)) {}

    motors_coast();
}

/**
 * @brief  Immediately stop all motion.
 */
void motion_stop(void)
{
    s_state       = MOTION_IDLE;
    s_motion_done = true;
    motors_brake();
    HAL_Delay(TURN_SETTLE_MS);
    motors_coast();
    motors_disable();
}

/* =========================================================================
 * PUBLIC API — RUNTIME CONTROL
 * ======================================================================= */

/**
 * @brief  Override cruise speed mid-move.
 */
void motion_set_speed(float speed_mmps)
{
    s_target_speed = clamp_f(speed_mmps, SPD_CREEP, SPD_ABSOLUTE_MAX);
}

/* =========================================================================
 * PUBLIC API — 1 KHz ISR ENTRY POINT
 * ======================================================================= */

/**
 * @brief  1 kHz control loop tick — called from TIM5 ISR in main.c.
 *
 * @details Top-level structure:
 *          1. Update sensors (IR + IMU) — always, every tick.
 *          2. Read encoder deltas — always.
 *          3. Dispatch to state handler.
 *
 *          Note on sensor ordering:
 *          ir_update() is called BEFORE the encoder reads because it
 *          takes ~720 µs (emitter pulses + settle times).  During that
 *          720 µs the encoders continue counting.  Reading encoders AFTER
 *          ir_update() gives values that correspond to the same 1ms
 *          time window as the sensor readings.
 */
void motion_1khz_tick(void)
{
    /* ── 1. Sensor updates ──────────────────────────────────────── */
    ir_update();        /* ~720 µs — fires all 6 emitter pairs       */
    imu_update();       /* ~20  µs — reads gyro Z via I2C, integrates*/

    /* ── 2. Encoder delta reads ─────────────────────────────────── */
    /* enc_left/right_delta() cache previous counts internally.      */
    /* These calls also update the speed estimate used by Layer 1.   */
    (void)enc_left_delta();
    (void)enc_right_delta();

    /* ── 3. FSM dispatch ─────────────────────────────────────────── */
    switch (s_state)
    {
        case MOTION_FORWARD:
            handle_forward();
            break;

        case MOTION_TURNING:
            handle_turning();
            break;

        case MOTION_ALIGNING:
            handle_aligning();
            break;

        case MOTION_BRAKING:
            handle_braking();
            break;

        case MOTION_IDLE:
        default:
            motors_stop();
            break;
    }
}

/* =========================================================================
 * PUBLIC API — STATUS / DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Return the current FSM state.
 */
MotionState_t motion_get_state(void)
{
    return s_state;
}

/**
 * @brief  Return true when the FSM is idle.
 */
bool motion_is_idle(void)
{
    return s_state == MOTION_IDLE;
}

/**
 * @brief  Populate a full motion status snapshot.
 */
void motion_get_status(MotionStatus_t *status)
{
    if (status == NULL) { return; }

    status->state           = s_state;
    status->target_speed    = s_target_speed;
    status->ramp_speed      = s_ramp_speed;
    status->meas_speed_l    = s_meas_spd_l;
    status->meas_speed_r    = s_meas_spd_r;
    status->position_mm     = enc_avg_mm();
    status->target_dist_mm  = s_target_dist_mm;
    status->yaw_deg         = imu_yaw_deg();
    status->turn_target_deg = s_turn_target_deg;
    status->tracking_err_mm = enc_tracking_error_mm();
    status->is_done         = s_motion_done;
}

/**
 * @brief  Return most recent measured left wheel speed.
 */
float motion_speed_left_mmps(void)
{
    return s_meas_spd_l;
}

/**
 * @brief  Return most recent measured right wheel speed.
 */
float motion_speed_right_mmps(void)
{
    return s_meas_spd_r;
}
