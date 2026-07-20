/**
 * @file    floodfill.h
 * @brief   Flood-fill distance map — public API.
 *
 * @details Implements the flood-fill algorithm that assigns every cell
 *          in the maze a distance value equal to the minimum number of
 *          cells that must be traversed to reach the goal from that cell.
 *          The robot then navigates by always moving to the adjacent cell
 *          with the lowest distance value.
 *
 *          WHAT FLOOD-FILL MEANS
 *          ──────────────────────
 *          Imagine pouring water into the 4-cell goal area.  Water flows
 *          through open passages (no wall blocking) and fills cells one
 *          by one.  The first cells to fill are those adjacent to the goal
 *          (distance 1), then their neighbours (distance 2), and so on.
 *          Every reachable cell eventually gets a distance value.
 *          Unreachable cells (completely walled off) stay at 255.
 *
 *          ALGORITHM: BREADTH-FIRST SEARCH (BFS)
 *          ──────────────────────────────────────
 *          BFS from the goal cells outward guarantees that every cell's
 *          distance is the shortest possible path length through the
 *          known wall map.  It runs in O(N) time where N = MAZE_SIZE²
 *          = 256 cells.  On a 100 MHz Cortex-M4 this takes < 0.1 ms —
 *          fast enough to run after every single wall update during search.
 *
 *          TWO FLOOD MODES
 *          ────────────────
 *          FLOOD_TO_GOAL   : Standard mode — seeds from the 4 goal cells.
 *                            Used during search to navigate toward the goal.
 *                            Used during speed runs to navigate the path.
 *
 *          FLOOD_TO_START  : Seeds from the start cell (15, 0) instead.
 *                            Used during return navigation after reaching goal.
 *                            Robot descends the gradient back to (15, 0).
 *
 *          NAVIGATION DECISION
 *          ────────────────────
 *          At each cell the robot calls floodfill_best_direction() which:
 *          1. Inspects all 4 adjacent cells.
 *          2. Eliminates those blocked by a wall in the current cell.
 *          3. Returns the direction toward the adjacent cell with the
 *             lowest flood distance.
 *          Ties are broken by direction preference (N > E > S > W) to
 *          give consistent behaviour when multiple optimal paths exist.
 *
 *          INCREMENTAL UPDATES DURING SEARCH
 *          ───────────────────────────────────
 *          Every time the robot discovers a new wall, floodfill_run()
 *          must be called again to recompute the entire distance map.
 *          This is correct because a newly discovered wall can invalidate
 *          the path the robot was following — a previously "short" path
 *          may now be blocked and a longer route must be chosen.
 *          Full recompute is cheaper than incremental update on a 16×16
 *          map — BFS over 256 cells is < 0.1 ms.
 *
 *          RELATIONSHIP WITH maze.c
 *          ─────────────────────────
 *          floodfill.c reads maze.walls[][] (via maze_has_wall() or
 *          maze_can_move()) to determine which passages are open.
 *          floodfill.c never writes to maze.walls[][].
 *          The distance map is stored separately in this module —
 *          maze.c has no knowledge of distances.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          maze.h   — MazeState_t, maze_can_move(), MAZE_DR/DC,
 *                     MAZE_WALL_FROM, maze_is_goal(), maze_is_start()
 *          config.h — MAZE_SIZE, GOAL_xxx, START_ROW, START_COL
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef FLOODFILL_H
#define FLOODFILL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "error.h"

/* =========================================================================
 * CONSTANTS
 * ======================================================================= */

/** Value assigned to unreachable cells (or unvisited cells in sparse mode). */
#define FLOOD_UNREACHABLE   255U

/** Value of goal cells in FLOOD_TO_GOAL mode. */
#define FLOOD_GOAL_DIST     0U

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  Flood-fill seeding mode.
 */
typedef enum
{
    FLOOD_TO_GOAL  = 0U,   /**< Seed from 4-cell goal area (standard)     */
    FLOOD_TO_START = 1U,   /**< Seed from start cell (return navigation)  */
} FloodMode_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the flood-fill module.
 *
 * @details Sets all distance values to FLOOD_UNREACHABLE (255).
 *          Does NOT run the flood-fill — call floodfill_run() after
 *          maze_init() to compute the first distance map.
 *
 * @return MM_OK always.
 */
MmResult_t floodfill_init(void);

/* =========================================================================
 * CORE ALGORITHM
 * ======================================================================= */

/**
 * @brief  Run the complete flood-fill BFS from seeds to all reachable cells.
 *
 * @details Resets all distances to FLOOD_UNREACHABLE, seeds the start
 *          cell(s) with distance 0, then expands outward through open
 *          passages using a FIFO queue (BFS).
 *
 *          After this returns:
 *          - Every cell reachable from the seed has a finite distance.
 *          - Unreachable cells remain at FLOOD_UNREACHABLE (255).
 *          - Calling floodfill_best_direction() is valid from any cell.
 *
 *          Execution time: < 0.1 ms at 100 MHz for a 16×16 maze.
 *          Call after every wall update during search.
 *
 * @param  mode  FLOOD_TO_GOAL  = navigate toward goal cells.
 *               FLOOD_TO_START = navigate toward start cell.
 */
void floodfill_run(FloodMode_t mode);

/**
 * @brief  Choose the best direction to move from (row, col).
 *
 * @details Examines all 4 directions.  For each direction:
 *          1. Checks maze_can_move(row, col, dir) — wall + bounds.
 *          2. Reads the flood distance of the neighbour.
 *          3. Selects the direction with the lowest distance.
 *          Ties broken by direction order: N preferred over E over S over W.
 *
 *          Returns DIR_NONE (0xFF) when no valid move exists (cell is a
 *          dead-end with no open passages).  explorer.c must handle this.
 *
 * @param  row  Current robot row.
 * @param  col  Current robot col.
 * @return uint8_t  Best direction (DIR_N/E/S/W) or DIR_NONE.
 */
uint8_t floodfill_best_direction(uint8_t row, uint8_t col);

/**
 * @brief  Return the flood distance of a specific cell.
 *
 * @details Returns FLOOD_UNREACHABLE (255) for out-of-bounds cells or
 *          cells not yet reached by the BFS.
 *
 * @param  row  Cell row.
 * @param  col  Cell col.
 * @return uint8_t  Distance value [0, 255].
 */
uint8_t floodfill_get_distance(uint8_t row, uint8_t col);

/**
 * @brief  Return a read-only pointer to the full 16×16 distance map.
 *
 * @details Allows maze.c (maze_print) and diagnostics.c to read the
 *          distance map without going through individual get_distance()
 *          calls.  The pointer is valid until the next floodfill_run().
 *
 * @return const uint8_t*  Pointer to flood[0][0] — row-major 16×16 array.
 */
const uint8_t *floodfill_get_map(void);

/* =========================================================================
 * PATH EXTRACTION
 * ======================================================================= */

/**
 * @brief  Extract the optimal path from start to goal as a sequence of
 *         directions, storing it in a caller-provided buffer.
 *
 * @details Runs FLOOD_TO_GOAL mode, then traces from (START_ROW, START_COL)
 *          to the nearest goal cell by always stepping to the neighbour
 *          with the lowest flood distance.
 *
 *          The path is stored as a sequence of DIR_N/E/S/W values.
 *          Terminates when a goal cell is reached or buf_len is exhausted.
 *
 *          Use in speedrun.c to pre-compute the optimal sequence of moves
 *          before the speed run begins, so the speed run executes the
 *          sequence without re-running flood-fill at every cell.
 *
 * @param[out] buf      Buffer to fill with direction sequence.
 * @param[in]  buf_len  Size of buf (max path length to extract).
 * @return uint8_t      Number of steps written. 0 = no path found.
 */
uint8_t floodfill_extract_path(uint8_t *buf, uint8_t buf_len);

/**
 * @brief  Count the number of steps in the optimal path to the goal.
 *
 * @details Equivalent to floodfill_get_distance(START_ROW, START_COL)
 *          after running FLOOD_TO_GOAL — the flood distance of the start
 *          cell is the optimal path length.
 *
 * @return uint8_t  Optimal step count. FLOOD_UNREACHABLE if no path exists.
 */
uint8_t floodfill_path_length(void);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Return the number of cells with a finite distance (reachable).
 *
 * @details Counts cells where distance < FLOOD_UNREACHABLE.
 *          Useful for monitoring search progress — when reachable_count
 *          stops increasing between flood runs, all accessible cells
 *          have been discovered.
 *
 * @return uint16_t  Number of reachable cells [0, 256].
 */
uint16_t floodfill_reachable_count(void);

/**
 * @brief  Return true when the goal is reachable from the start.
 *
 * @details Checks whether floodfill_get_distance(START_ROW, START_COL)
 *          is less than FLOOD_UNREACHABLE after a FLOOD_TO_GOAL run.
 *
 * @return bool  true = a path to the goal exists.
 */
bool floodfill_goal_reachable(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOODFILL_H */
