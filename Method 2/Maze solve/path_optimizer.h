/**
 * @file    path_optimizer.h
 * @brief   Path optimizer — public API.
 *
 * @details Transforms the raw direction sequence produced by floodfill.c
 *          into an optimised move sequence that is faster to execute
 *          during speed runs.
 *
 *          WHAT OPTIMISATION MEANS HERE
 *          ─────────────────────────────
 *          The flood-fill path is a sequence of per-cell directions:
 *            [N, N, E, E, E, N, N, N, W, W]
 *          Each direction is one 180mm cell.  If executed naively, the
 *          robot stops at every cell boundary, turns if needed, then
 *          moves one cell — including stopping after straight runs.
 *          This wastes time and creates unnecessary deceleration/acceleration
 *          cycles that stress the motors.
 *
 *          The optimizer converts this into a run-length encoded sequence:
 *            [FORWARD 2, TURN_RIGHT, FORWARD 3, TURN_LEFT (×2), FORWARD 3,
 *             TURN_LEFT (×2), FORWARD 2]
 *          Wait — let's use the real example:
 *            [N×2, E×3, N×3, W×2] →
 *            [FORWARD 2, TURN_RIGHT 90, FORWARD 3, TURN_LEFT 90,
 *             FORWARD 3, TURN_LEFT 90, TURN_LEFT 90 ... ]
 *          Actually a U-turn is just TURN_180.  The optimizer handles
 *          TURN_RIGHT (90°CW), TURN_LEFT (90°CCW), and TURN_180.
 *
 *          MULTI-CELL STRAIGHT MERGING
 *          ────────────────────────────
 *          The most important optimisation.  Consecutive cells in the same
 *          direction are merged into a single move_forward(N, speed) call.
 *          This allows the velocity profile to reach full cruise speed over
 *          multiple cells without interruption.
 *
 *          Example: 5 consecutive North cells at SPD_RUN3 (750 mm/s):
 *            Naive:    5 × [accel, cruise, decel] = 5 × ~0.8 s = 4.0 s
 *            Optimised: [accel, cruise×4, decel]  =     ~1.5 s total
 *            Saving: ~2.5 s per 5-cell straight
 *
 *          TURN ENCODING
 *          ──────────────
 *          A turn is inserted whenever consecutive directions differ.
 *          The turn angle is computed from the heading change:
 *            current → next:
 *              N→E: +90° (right)     E→S: +90° (right)
 *              S→W: +90° (right)     W→N: +90° (right)
 *              N→W: −90° (left)      W→S: −90° (left)
 *              S→E: −90° (left)      E→N: −90° (left)
 *              N→S: ±180° (U-turn)   (and E→W, S→N, W→E)
 *
 *          OUTPUT MOVE SEQUENCE
 *          ─────────────────────
 *          The optimizer produces an array of PathMove_t structs:
 *            { MOVE_FORWARD,   cells=3, turn=0   }
 *            { MOVE_TURN_RIGHT,cells=0, turn=90  }
 *            { MOVE_FORWARD,   cells=2, turn=0   }
 *          speedrun.c iterates this array and calls the appropriate
 *          motion.c function for each entry.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          maze.h   — DIR_N/E/S/W, NUM_DIRS
 *          config.h — MAZE_SIZE, CELL_WIDTH_MM, SPD_xxx
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H

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

/**
 * @brief  Maximum number of moves in an optimised path.
 *
 * @details Worst case: a path that turns every cell.
 *          In a 16×16 maze the longest path is ≤ 256 cells.
 *          Each cell could generate at most 1 turn + 1 forward,
 *          so 512 entries is a safe upper bound.
 *          In practice, an optimised path in a typical maze has
 *          20–60 entries.
 */
#define PATH_MAX_MOVES  128U

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  Move type in the optimised path.
 */
typedef enum
{
    MOVE_FORWARD    = 0U,  /**< Drive forward N cells (cells field used)   */
    MOVE_TURN_RIGHT = 1U,  /**< Pivot 90° clockwise                       */
    MOVE_TURN_LEFT  = 2U,  /**< Pivot 90° counter-clockwise               */
    MOVE_TURN_180   = 3U,  /**< Pivot 180° U-turn                         */
    MOVE_DONE       = 4U,  /**< Sentinel — end of path marker             */
} MoveType_t;

/**
 * @brief  Single move entry in the optimised path.
 */
typedef struct
{
    MoveType_t type;    /**< What kind of move this is                    */
    uint8_t    cells;   /**< For MOVE_FORWARD: number of cells (1–16)     */
                        /**< For turns: unused (set to 0)                  */
    float      speed;   /**< Target speed for MOVE_FORWARD (mm/s)         */
                        /**< Set by path_optimizer_run() from config.h    */
} PathMove_t;

/**
 * @brief  Complete optimised path — array of moves plus count.
 */
typedef struct
{
    PathMove_t moves[PATH_MAX_MOVES];  /**< Ordered move sequence          */
    uint8_t    count;                  /**< Number of valid entries in moves*/
    uint8_t    total_cells;            /**< Total cells in the path        */
    uint8_t    total_turns;            /**< Total turn count               */
    float      run_speed;              /**< Speed used for FORWARD moves   */
} OptPath_t;

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the path optimizer module.
 *
 * @details Clears the internal optimised path buffer.
 *          Call once during system bring-up.
 *
 * @return MM_OK always.
 */
MmResult_t path_optimizer_init(void);

/* =========================================================================
 * OPTIMISATION
 * ======================================================================= */

/**
 * @brief  Build the optimised move sequence from a direction path.
 *
 * @details Takes the raw direction sequence from floodfill_extract_path()
 *          and converts it to an optimised PathMove_t sequence by:
 *
 *          1. RUN-LENGTH ENCODING OF STRAIGHTS
 *             Consecutive identical directions are merged into one
 *             MOVE_FORWARD entry with cells = run length.
 *
 *          2. TURN INSERTION
 *             When direction changes, compute the required turn
 *             (TURN_RIGHT / TURN_LEFT / TURN_180) and insert it.
 *
 *          3. SPEED ASSIGNMENT
 *             Each MOVE_FORWARD gets speed = run_speed parameter.
 *             Turn entries get speed = 0 (motion.c uses TURN_SPD internally).
 *
 *          4. DONE SENTINEL
 *             A MOVE_DONE entry is appended after the last move.
 *
 *          Example input (raw directions):
 *            [N, N, N, E, E, S, S, S, S]   (9 cells)
 *
 *          Example output (optimised):
 *            FORWARD(3)  TURN_RIGHT  FORWARD(2)  TURN_RIGHT  FORWARD(4)
 *            DONE
 *
 *          The output is stored in the internal OptPath_t.
 *          Call path_optimizer_get() to access the result.
 *
 * @param[in] dirs      Array of direction values (DIR_N/E/S/W).
 *                      Produced by floodfill_extract_path().
 * @param[in] dir_count Number of entries in dirs (path length in cells).
 * @param[in] run_speed Speed to assign to all MOVE_FORWARD entries (mm/s).
 *                      Use SPD_RUN1, SPD_RUN2, SPD_RUN3 from config.h.
 *
 * @return MM_OK              Optimisation succeeded.
 * @return MM_ERR_PARAM       dirs is NULL or dir_count is 0.
 * @return MM_ERR_OVERFLOW    Path is too long for PATH_MAX_MOVES buffer.
 */
MmResult_t path_optimizer_run(const uint8_t *dirs,
                               uint8_t        dir_count,
                               float          run_speed);

/**
 * @brief  Return a read-only pointer to the most recently optimised path.
 *
 * @details Valid after path_optimizer_run() returns MM_OK.
 *          The pointer is stable until the next call to path_optimizer_run().
 *
 * @return const OptPath_t*  Pointer to the optimised path. Never NULL.
 */
const OptPath_t *path_optimizer_get(void);

/* =========================================================================
 * INDIVIDUAL OPERATIONS
 * ======================================================================= */

/**
 * @brief  Compute the turn type required to change from one heading to another.
 *
 * @details Heading convention: DIR_N=0, DIR_E=1, DIR_S=2, DIR_W=3.
 *          Turn type from current heading to next heading:
 *            diff = (next − current + 4) % 4
 *            diff == 1 → TURN_RIGHT
 *            diff == 3 → TURN_LEFT
 *            diff == 2 → TURN_180
 *            diff == 0 → no turn (same heading — should not occur in path)
 *
 * @param  from_dir  Current heading (DIR_N/E/S/W).
 * @param  to_dir    Target  heading (DIR_N/E/S/W).
 * @return MoveType_t  MOVE_TURN_RIGHT / MOVE_TURN_LEFT / MOVE_TURN_180.
 *                     Returns MOVE_DONE if from_dir == to_dir.
 */
MoveType_t path_optimizer_turn_type(uint8_t from_dir, uint8_t to_dir);

/* =========================================================================
 * DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Print the optimised path as human-readable text via UART logger.
 *
 * @details Outputs one line per move:
 *            "FORWARD  3 cells  @ 750 mm/s"
 *            "TURN_RIGHT"
 *            "FORWARD  2 cells  @ 750 mm/s"
 *            ...
 *            "DONE  (5 moves, 5 cells, 1 turn)"
 *
 * @param[in] path  OptPath_t to print. NULL = print last optimised path.
 */
void path_optimizer_print(const OptPath_t *path);

#ifdef __cplusplus
}
#endif

#endif /* PATH_OPTIMIZER_H */
