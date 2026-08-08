/**
 * @file    speedrun.c
 * @brief   MicroMaze 3 · Execute the optimised path at speed — implementation.
 * @details See speedrun.h for the full design rationale, including why
 *          this is a pure open-loop executor with no per-cell sensing.
 *
 * @author  VDawn
 * @date    2026
 */
#include "speedrun.h"
#include "config.h"
#include "error.h"
#include "logger.h"
#include "maze.h"
#include "floodfill.h"
#include "path_optimizer.h"
#include "turn.h"
#include "safety.h"
#include "oled.h"
#include "buzzer.h"
#include "main.h"     /* HAL_GetTick */

/** Raw direction-sequence buffer. floodfill_extract_path()'s buf_len
 *  parameter is uint8_t (max 255) — NOT MAZE_SIZE*MAZE_SIZE (256),
 *  which would silently truncate to 0 when passed as a uint8_t
 *  argument and make every call report "no path found". 255 is also
 *  the correct theoretical bound regardless: a simple (no-revisit)
 *  path through all 256 cells has exactly 255 steps between them. */
#define SPEEDRUN_DIR_BUF_LEN  255U

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t speedrun_init(void)
{
    return MM_OK;
}

MmResult_t speedrun_run(float speed_mmps)
{
    LOG_INFO("Speedrun: starting at %.0f mm/s", speed_mmps);

    uint8_t dirs[SPEEDRUN_DIR_BUF_LEN];
    uint8_t dir_count = floodfill_extract_path(dirs, SPEEDRUN_DIR_BUF_LEN);

    if (dir_count == 0U)
    {
        LOG_ERROR("Speedrun: no path to goal in the known map — run a search first");
        oled_show_message("SPEEDRUN", "NO PATH KNOWN");
        return MM_ERR_NOT_FOUND;
    }

    MmResult_t result = path_optimizer_run(dirs, dir_count, speed_mmps);
    if (result != MM_OK)
    {
        LOG_ERROR("Speedrun: path optimizer failed: %d", (int)result);
        oled_show_error("PATH OPT FAIL", (int32_t)result);
        return result;
    }

    const OptPath_t *path = path_optimizer_get();

    /* The robot is assumed to physically be at the start cell — sync
     * both position trackers before executing. */
    maze_set_position(START_ROW, START_COL, DIR_N);
    turn_set_heading(DIR_N);

    oled_show_message("SPEEDRUN", "GO");

    uint32_t start_ms   = HAL_GetTick();
    result               = turn_execute_path(path);
    uint32_t elapsed_ms  = HAL_GetTick() - start_ms;

    if (result == MM_OK)
    {
        LOG_INFO("Speedrun: complete in %lu ms", (unsigned long)elapsed_ms);
        oled_clear();
        oled_linef(0U, "SPEEDRUN DONE");
        oled_linef(1U, "TIME: %lu ms", (unsigned long)elapsed_ms);
        (void)oled_flush();
        buzzer_goal();
    }
    else
    {
        LOG_ERROR("Speedrun: aborted after %lu ms — safety trip (reason %d)",
                  (unsigned long)elapsed_ms, (int)safety_trip_reason());
        oled_show_error("SPEEDRUN ABORT", (int32_t)result);
    }

    return result;
}
