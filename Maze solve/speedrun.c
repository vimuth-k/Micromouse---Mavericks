/**
 * @file    speedrun.c
 * @brief   MicroMaze 3 · Execute the optimised path at speed — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "speedrun.h"
#include "config.h"
#include "error.h"
#include "maze.h"
#include "floodfill.h"
#include "path_optimizer.h"
#include "turn.h"
#include "safety.h"
#include "oled.h"
#include "main.h"

#define SPEEDRUN_DIR_BUF_LEN  255U

MmResult_t speedrun_run(float speed_mmps)
{
    uint8_t dirs[SPEEDRUN_DIR_BUF_LEN];
    uint8_t dir_count = floodfill_extract_path(dirs, SPEEDRUN_DIR_BUF_LEN);

    if (dir_count == 0U)
    {
        oled_show_message("SPEEDRUN", "NO PATH KNOWN");
        return MM_ERR_NOT_FOUND;
    }

    MmResult_t result = path_optimizer_run(dirs, dir_count, speed_mmps);
    if (result != MM_OK)
    {
        oled_show_error("PATH OPT FAIL", (int32_t)result);
        return result;
    }

    const OptPath_t *path = path_optimizer_get();

    maze_set_position(START_ROW, START_COL, DIR_N);
    turn_set_heading(DIR_N);

    oled_show_message("SPEEDRUN", "GO");

    uint32_t start_ms   = HAL_GetTick();
    result               = turn_execute_path(path);
    uint32_t elapsed_ms  = HAL_GetTick() - start_ms;

    if (result == MM_OK)
    {
        oled_clear();
        oled_linef(0U, "SPEEDRUN DONE");
        oled_linef(1U, "TIME: %lu ms", (unsigned long)elapsed_ms);
        (void)oled_flush();
    }
    else
    {
        oled_show_error("SPEEDRUN ABORT", (int32_t)result);
    }

    return result;
}
