/**
 * @file    motion.h
 * @brief   Motion control — public API.
 *
 * @details This module is the bridge between the navigation layer
 *          (maze.c, explorer.c) and the hardware layer (motors.c,
 *          encoders.c, ir.c, imu.c).  It provides five blocking
 *          primitives that the navigation layer calls:
 *
 *            move_forward()       drive N cells forward
 *            motion_turn_right()  pivot 90° clockwise
 *            motion_turn_left()   pivot 90° counter-clockwise
 *            motion_turn_180()    pivot 180° U-turn
 *            motion_align_front() square up to a front wall
 *
 *          All five block until the manoeuvre is complete, then return.
 *          Navigation code reads as a simple sequence of commands:
 *
 *            move_forward(1, SPD_SEARCH);
 *            motion_turn_right();
 *            move_forward(2, SPD_SEARCH);
 *
 *          THE 1 KHz CONTROL LOOP
 *          ───────────────────────
 *          motion_1khz_tick() is called from the TIM5 interrupt every
 *          1 ms.  It runs a layered control structure:
 *
 *            Layer 1 — speed PID (per wheel)
 *              Error  : target_speed − measured_speed (mm/s)
 *              Output : base PWM command (counts)
 *
 *            Layer 2 — straightness PID (encoder difference)
 *              Error  : enc_left_count − enc_right_count (counts)
 *              Output : ± correction added to left/right PWM
 *
 *            Layer 3 — heading PID (gyro yaw)
 *              Error  : 0° − gyro_yaw_deg (during straights)
 *              Output : ± correction added to left/right PWM
 *
 *            Layer 4 — wall centering (IR side sensors)
 *              Error  : ir_side_error() (ADC count difference)
 *              Output : ± correction added to left/right PWM
 *              Active : only when both side walls are present
 *
 *          The four corrections combine additively:
 *            pwm_left  = speed_pid_left  − straight − heading − wall
 *            pwm_right = speed_pid_right + straight + heading + wall
 *
 *          TRAPEZOIDAL VELOCITY PROFILE
 *          ──────────────────────────────
 *          move_forward() uses a trapezoidal profile — the target speed
 *          ramps from 0 → cruise speed at ACCEL mm/s², holds at cruise,
 *          then ramps from cruise → 0 at DECEL mm/s² starting when the
 *          remaining distance equals the braking distance:
 *            brake_dist = v² / (2 × DECEL)
 *          This ensures the robot stops precisely at the cell boundary
 *          regardless of speed.  The speed PID tracks the ramp profile,
 *          not a step input — this prevents instantaneous demand spikes.
 *
 *          BLOCKING MECHANISM
 *          ───────────────────
 *          The blocking primitives (move_forward, turn, align) set a
 *          volatile flag (s_motion_done) and enter a spin-wait loop.
 *          The TIM5 ISR sets the flag when the manoeuvre completes.
 *          The spin-wait is:
 *            while (!s_motion_done) {}
 *          This is safe because the ISR is the only writer.  The main
 *          loop has nothing useful to do while waiting anyway.
 *
 *          TURN ALGORITHM
 *          ───────────────
 *          Turns are purely gyro-based (pivot in place, left+right
 *          motors at equal speed opposite directions):
 *            1. Reset gyro yaw to 0°.
 *            2. Set turn_target to ±90° or ±180°.
 *            3. ISR runs turn PID: error = turn_target − yaw.
 *            4. When |error| < TURN_TOLERANCE_DEG, apply brake.
 *            5. Wait TURN_SETTLE_MS for oscillation to damp.
 *            6. Reset gyro yaw again (zero for next straight move).
 *
 *          DEPENDENCIES
 *          ─────────────
 *          Control/ : pid.h
 *          Drivers/ : motors.h, encoders.h, ir.h, imu.h
 *          config.h : all SPD_xxx, ACCEL_xxx, KP_xxx, TURN_xxx constants
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef MOTION_H
#define MOTION_H

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
 * @brief  Motion state machine states.
 *
 * @details The ISR reads s_state to determine what to compute.
 *          Exported here so diagnostics.c can display the current state.
 */
typedef enum
{
    MOTION_IDLE      = 0U,  /**< Motors stopped, waiting for next command  */
    MOTION_FORWARD   = 1U,  /**< Driving forward with velocity profile      */
    MOTION_TURNING   = 2U,  /**< Pivot turn — gyro feedback only           */
    MOTION_ALIGNING  = 3U,  /**< Nudging toward front wall for alignment   */
    MOTION_BRAKING   = 4U,  /**< Applying active brake before going IDLE   */
} MotionState_t;

/**
 * @brief  Snapshot of current motion status for diagnostics.
 */
typedef struct
{
    MotionState_t state;           /**< Current FSM state                  */
    float         target_speed;    /**< Commanded cruise speed (mm/s)      */
    float         ramp_speed;      /**< Current ramp profile speed (mm/s)  */
    float         meas_speed_l;    /**< Measured left  wheel speed (mm/s)  */
    float         meas_speed_r;    /**< Measured right wheel speed (mm/s)  */
    float         position_mm;     /**< Distance from move start (mm)      */
    float         target_dist_mm;  /**< Total distance for current move    */
    float         yaw_deg;         /**< Current gyro heading (degrees)     */
    float         turn_target_deg; /**< Turn target angle (degrees)        */
    float         tracking_err_mm; /**< Encoder tracking error (mm)        */
    bool          is_done;         /**< True when current move is complete */
} MotionStatus_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the motion control module.
 *
 * @details Creates and configures all five PID instances:
 *            pid_speed_left   — KP/KI/KD_SPEED,    SPEED_INTEG_LIMIT,    PWM_MAX
 *            pid_speed_right  — KP/KI/KD_SPEED,    SPEED_INTEG_LIMIT,    PWM_MAX
 *            pid_straight     — KP/KI/KD_STRAIGHT,  STRAIGHT_INTEG_LIMIT, STRAIGHT_OUTPUT_LIMIT
 *            pid_heading      — KP/KI/KD_HEADING,   HEADING_INTEG_LIMIT,  HEADING_OUTPUT_LIMIT
 *            pid_turn         — KP/KI/KD_TURN,      TURN_INTEG_LIMIT,     TURN_PWM_MAX
 *          Sets FSM state to MOTION_IDLE.
 *          Call once during system bring-up after encoders_init() and
 *          imu_init() have completed.
 *
 * @return MM_OK always.
 */
MmResult_t motion_init(void);

/* =========================================================================
 * BLOCKING MOVE PRIMITIVES
 * All functions below block until the manoeuvre is complete.
 * Safe to call from the main loop only — never from the 1 kHz ISR.
 * ======================================================================= */

/**
 * @brief  Drive forward N cells using a trapezoidal velocity profile.
 *
 * @details Resets encoders and gyro yaw to zero, configures the velocity
 *          profile for (cells × CELL_WIDTH_MM) at speed_mmps, sets the
 *          FSM to MOTION_FORWARD, and blocks until the profile completes.
 *
 *          The ISR runs layers 1–4 (speed + straight + heading + wall PIDs)
 *          on every tick.  The profile ramps speed up and down automatically.
 *          When enc_avg_mm() reaches target_dist_mm the ISR transitions
 *          to MOTION_BRAKING then MOTION_IDLE and sets s_motion_done.
 *
 *          After this function returns:
 *            - Motors are stopped (coast).
 *            - Encoders reflect the distance just travelled.
 *            - Gyro yaw is the accumulated drift from that straight.
 *
 * @param  cells       Number of 180 mm cells to travel (1–16 typical).
 * @param  speed_mmps  Target cruise speed (mm/s). Use SPD_SEARCH,
 *                     SPD_RUN1, SPD_RUN2, SPD_RUN3 from config.h.
 */
void move_forward(uint8_t cells, float speed_mmps);

/**
 * @brief  Pivot turn 90° clockwise (right).
 *
 * @details Resets gyro yaw, sets turn_target = +90°, runs turn PID
 *          in ISR until |yaw − 90°| < TURN_TOLERANCE_DEG, applies
 *          brake, waits TURN_SETTLE_MS, resets gyro yaw for next move.
 *          Blocks until settled.
 */
void motion_turn_right(void);

/**
 * @brief  Pivot turn 90° counter-clockwise (left).
 *
 * @details Same as motion_turn_right() with turn_target = −90°.
 */
void motion_turn_left(void);

/**
 * @brief  Pivot turn 180° (U-turn).
 *
 * @details turn_target = +180°.  Robot ends up facing the opposite
 *          direction from its entry heading.
 */
void motion_turn_180(void);

/**
 * @brief  Nudge forward to square up against the front wall.
 *
 * @details Only executes if ir_wall_front() is true.  If no front wall
 *          is detected, returns immediately (no-op).
 *
 *          When a front wall is present:
 *            1. Sets FSM to MOTION_ALIGNING.
 *            2. ISR slowly drives forward (~400 PWM per wheel).
 *            3. Front wall error (ir_front_error()) steers to equalise
 *               FL and FR readings (robot squares up to wall).
 *            4. When ir_wall_front_close() is true AND |front_error|
 *               < 40 ADC counts, transition to MOTION_BRAKING.
 *            5. Returns after brake settle.
 *
 *          Call before every turn to improve turn accuracy.  A robot
 *          that is 3 mm off-centre in a cell accumulates ~0.95° heading
 *          error per turn — after 4 turns (one complete loop) this is
 *          ~3.8°, enough to clip a wall at speed.
 */
void motion_align_front(void);

/**
 * @brief  Immediately stop both motors (active brake then coast).
 *
 * @details Forces FSM to MOTION_IDLE.  Applies motors_brake() for
 *          TURN_SETTLE_MS then motors_coast().  Safe to call at any
 *          time including from the safety module.
 */
void motion_stop(void);

/* =========================================================================
 * RUNTIME CONTROL
 * ======================================================================= */

/**
 * @brief  Override the current cruise speed target during a move.
 *
 * @details Allows the speed to be changed mid-move without restarting.
 *          The velocity ramp will start from the current ramp speed and
 *          ramp to the new target at ACCEL/DECEL rates.
 *          Used by speedrun.c to progressively increase speed if the
 *          robot successfully completes cells without wall contact.
 *
 * @param  speed_mmps  New target speed. Clamped to [SPD_CREEP, SPD_ABSOLUTE_MAX].
 */
void motion_set_speed(float speed_mmps);

/* =========================================================================
 * 1 KHz ISR ENTRY POINT
 * ======================================================================= */

/**
 * @brief  1 kHz control loop tick — call from TIM5 interrupt ONLY.
 *
 * @details This function must be called from HAL_TIM_PeriodElapsedCallback
 *          in main.c when htim->Instance == TIM5.
 *
 *          Execution time target: < 250 µs (budgeted per tick).
 *          Actual measured: ~195 µs at 100 MHz (sensors 120µs +
 *          gyro I2C 20µs + encoders 5µs + 5×PID 30µs + motor write 5µs).
 *
 *          DO NOT call from any other context.
 *          DO NOT call HAL_Delay() or any blocking function from this.
 *
 * @note   The sensors_update() and imu_update() calls are made here,
 *         not in separate ISR hooks, to guarantee that all measurements
 *         are taken at the same instant and are temporally consistent
 *         for each PID computation.
 */
void motion_1khz_tick(void);

/* =========================================================================
 * STATUS / DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Return the current FSM state.
 *
 * @return MotionState_t  Current state (IDLE/FORWARD/TURNING/etc.).
 */
MotionState_t motion_get_state(void);

/**
 * @brief  Return true when the motion FSM is idle (no move in progress).
 *
 * @return bool  true = motors stopped, ready for next command.
 */
bool motion_is_idle(void);

/**
 * @brief  Populate a full motion status snapshot.
 *
 * @details Captures all state for UART logging and OLED display without
 *          multiple individual function calls.  Not safe to call from
 *          the ISR — call from the main loop between ticks.
 *
 * @param[out] status  Pointer to MotionStatus_t to fill. Must not be NULL.
 */
void motion_get_status(MotionStatus_t *status);

/**
 * @brief  Return the most recently measured left wheel speed (mm/s).
 *
 * @details Cached by the ISR each tick.  Read from main loop for display.
 *
 * @return float  Speed mm/s. Positive = forward.
 */
float motion_speed_left_mmps(void);

/**
 * @brief  Return the most recently measured right wheel speed (mm/s).
 *
 * @return float  Speed mm/s. Positive = forward.
 */
float motion_speed_right_mmps(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */
