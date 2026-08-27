/**
 * @file    modes.c
 * @brief   MicroMaze 3 · DIP-mode dispatch and execution implementation.
 * @details Direct dispatcher for all operating modes. Subsystems are called
 *          directly or via static helpers, with states coordinated and a final
 *          idle loop entered upon completion.
 *
 * @author  VDawn
 * @date    2026
 */
#include "modes.h"
#include "config.h"
#include "error.h"
#include "oled.h"
#include "motors.h"
#include "encoders.h"
#include "motion.h"
#include "imu.h"
#include "turn.h"
#include "explorer.h"
#include "speedrun.h"
#include "wall_follow.h"
#include "calibration.h"
#include "diagnostics.h"
#include "startup_test.h"
#include "state_machine.h"
#include "scheduler.h"
#include "main.h"     /* HAL_Delay */
#include "pins.h"
#include <stdio.h>
/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */
/**
 * @brief  Show a final result on the OLED, then loop forever.
 *
 * @details Services the scheduler while idling so battery monitoring
 *          keeps running even after the mode's task is done. Only a
 *          board reset gets out of this loop.
 *
 * @param  line1  First OLED line (result headline).
 * @param  line2  Second OLED line (detail, e.g. a measurement or PASS/FAIL).
 */
static void modes_idle_forever(const char *line1, const char *line2)
{
    oled_show_message(line1, line2);
    while (1)
    {
        scheduler_tick();
        HAL_Delay(500U);
    }
}
/**
 * @brief  Spin one motor briefly at STARTUP_TEST_PWM. Shared by
 *         run_motor_test()'s left/right steps.
