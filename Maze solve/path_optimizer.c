/**
 * @file    path_optimizer.c
 * @brief   Path optimizer — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          Takes the raw per-cell direction sequence from flood-fill
 *          and converts it into the smallest possible sequence of
 *          robot motion commands.  The core insight is simple:
 *
 *            Raw:       N N N E E E E N N → 9 separate cell moves
 *            Optimised: FORWARD(3) TURN_RIGHT FORWARD(4) TURN_LEFT FORWARD(2)
 *                       → 5 motion commands, 2 stops instead of 8
 *
 *          At 750 mm/s, every unnecessary stop-and-restart cycle costs
 *          approximately 0.3–0.5 seconds (decel to zero + accel back to
 *          cruise).  In a maze with 8 straight runs of 3 cells each, the
 *          naive approach wastes up to 4 seconds compared to optimised.
 *          That 4 seconds is the difference between 1st and 3rd place.
 *
 *          ALGORITHM: SINGLE-PASS RUN-LENGTH ENCODING
 *          ──────────────────────────────────────────
 *          The optimizer makes one forward pass through the direction array:
 *
 *            current_heading = START_HEADING (DIR_N)
 *            straight_count  = 0
 *
 *            for each dir in path:
 *              if dir == current_heading:
 *                straight_count++              (extend current straight)
 *              else:
 *                emit FORWARD(straight_count)  (flush pending straight)
 *                emit TURN(current → dir)      (insert turn)
 *                current_heading = dir
 *                straight_count  = 1           (start new straight)
 *
 *            emit FORWARD(straight_count)      (flush final straight)
 *            emit DONE
 *
 *          This is O(N) in path length — one pass, no backtracking.
 *          For a 256-cell path this completes in under 10 µs.
 *
 *          TURN TYPE DERIVATION
 *          ─────────────────────
 *          Headings are DIR_N=0, DIR_E=1, DIR_S=2, DIR_W=3.
 *          The turn from current to next heading is:
 *            diff = (next - current + 4) % 4
 *            1 → right 90°    (clockwise)
 *            3 → left  90°    (counter-clockwise)
 *            2 → 180° U-turn
 *            0 → no turn (same direction, handled by straight merge)
 *
 *          This works because the headings are arranged clockwise:
 *          N(0) → E(1) → S(2) → W(3) → N(0) ...
 *          Adding 1 mod 4 rotates clockwise (right).
 *          Subtracting 1 mod 4 rotates counter-clockwise (left).
 *          A diff of 2 is a half rotation in either direction (U-turn).
 *
 *          INITIAL HEADING
 *          ────────────────
 *          The robot always starts at (START_ROW, START_COL) facing
 *          START_HEADING (DIR_N).  The first direction in the path array
 *          may or may not match this heading:
 *            - If it matches: the first straight begins immediately.
 *            - If it differs: a turn is inserted before the first FORWARD.
 *          This handles maze configurations where the optimal first move
 *          is not straight ahead from the start cell.
 *
 *          EDGE CASES HANDLED
 *          ───────────────────
 *          1. dir_count == 1: single-cell path → one FORWARD(1) + DONE.
 *          2. All same direction: one FORWARD(N) + DONE, zero turns.
 *          3. Alternating directions (maximum turns): N E N E N E...
 *             → FORWARD(1) TURN_RIGHT FORWARD(1) TURN_LEFT FORWARD(1)...
 *             Each single-cell straight gets its own FORWARD entry.
 *          4. U-turn in the middle: FORWARD(N) TURN_180 FORWARD(M).
 *          5. Path starts with a turn (first dir ≠ START_HEADING):
 *             TURN_xxx FORWARD(N) ...
 *
 *          OUTPUT VALIDATION
 *          ──────────────────
 *          After building the sequence, path_optimizer_run() verifies:
 *          - Total cells in FORWARD entries == dir_count.
 *          - move count <= PATH_MAX_MOVES.
 *          If either check fails, returns MM_ERR_OVERFLOW.
 *
 * @author  VDawn
 * @date    2026
 */

#include "path_optimizer.h"
#include "maze.h"
#include "config.h"
#include <string.h>

/* =========================================================================
 * PRIVATE — INTERNAL PATH STORAGE
 * ======================================================================= */

/** Single internal path instance — overwritten on each optimizer_run() call. */
static OptPath_t s_path;

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Append a move to the internal path buffer.
 *
 * @details Returns false if PATH_MAX_MOVES would be exceeded.
 *
 * @param  type   Move type.
 * @param  cells  Cell count (used for MOVE_FORWARD, 0 otherwise).
 * @param  speed  Forward speed in mm/s (used for MOVE_FORWARD, 0 otherwise).
 * @return bool   true = appended successfully.
 */
static bool append_move(MoveType_t type, uint8_t cells, float speed)
{
    if (s_path.count >= PATH_MAX_MOVES) { return false; }

    s_path.moves[s_path.count].type  = type;
    s_path.moves[s_path.count].cells = cells;
    s_path.moves[s_path.count].speed = speed;
    s_path.count++;

    return true;
}

/**
 * @brief  Flush a pending straight run into the path buffer.
 *
 * @details Only appends if run_len > 0.  Updates s_path.total_cells.
 *
 * @param  run_len  Number of cells in this straight.
 * @param  speed    Speed for this forward move.
 * @return bool     true = flushed successfully (or nothing to flush).
 */
static bool flush_straight(uint8_t run_len, float speed)
{
    if (run_len == 0U) { return true; }
    s_path.total_cells += run_len;
    return append_move(MOVE_FORWARD, run_len, speed);
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the path optimizer module.
 */
MmResult_t path_optimizer_init(void)
{
    (void)memset(&s_path, 0, sizeof(s_path));
    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — OPTIMISATION
 * ======================================================================= */

/**
 * @brief  Build the optimised move sequence from a direction path.
 */
MmResult_t path_optimizer_run(const uint8_t *dirs,
                               uint8_t        dir_count,
                               float          run_speed)
{
    if (dirs == NULL || dir_count == 0U) { return MM_ERR_PARAM; }

    /* ── Reset internal path ─────────────────────────────────────── */
    (void)memset(&s_path, 0, sizeof(s_path));
    s_path.run_speed = run_speed;

    /* ── Initial state ───────────────────────────────────────────── */
    uint8_t cur_heading   = (uint8_t)START_HEADING;   /* DIR_N */
    uint8_t straight_run  = 0U;

    /* ── Single-pass run-length encoding ─────────────────────────── */
    for (uint8_t i = 0U; i < dir_count; i++)
    {
        uint8_t dir = dirs[i];

        if (dir >= NUM_DIRS) { continue; }   /* Skip invalid entries */

        if (dir == cur_heading)
        {
            /* Same direction — extend current straight */
            straight_run++;
        }
        else
        {
            /* Direction changed:
               1. Flush any pending straight run.
               2. Insert the required turn.
               3. Start a new straight (this first cell of new direction). */

            if (!flush_straight(straight_run, run_speed))
            {
                return MM_ERR_OVERFLOW;
            }

            /* Compute and insert turn */
            MoveType_t turn = path_optimizer_turn_type(cur_heading, dir);
            if (turn != MOVE_DONE)
            {
                if (!append_move(turn, 0U, 0.0f))
                {
                    return MM_ERR_OVERFLOW;
                }
                s_path.total_turns++;
            }

            /* New heading, start counting */
            cur_heading  = dir;
            straight_run = 1U;
        }
    }

    /* ── Flush the final straight ─────────────────────────────────── */
    if (!flush_straight(straight_run, run_speed))
    {
        return MM_ERR_OVERFLOW;
    }

    /* ── Append DONE sentinel ─────────────────────────────────────── */
    if (!append_move(MOVE_DONE, 0U, 0.0f))
    {
        return MM_ERR_OVERFLOW;
    }

    /* ── Validate: total forward cells must equal input path length ── */
    if (s_path.total_cells != dir_count)
    {
        /* Internal consistency failure — should never happen */
        return MM_ERR_OVERFLOW;
    }

    return MM_OK;
}

/**
 * @brief  Return read-only pointer to the most recently optimised path.
 */
const OptPath_t *path_optimizer_get(void)
{
    return &s_path;
}

/* =========================================================================
 * PUBLIC API — INDIVIDUAL OPERATIONS
 * ======================================================================= */

/**
 * @brief  Compute the turn type required to change heading.
 *
 * @details diff = (to_dir - from_dir + 4) % 4:
 *            1 → MOVE_TURN_RIGHT  (clockwise 90°)
 *            3 → MOVE_TURN_LEFT   (counter-clockwise 90°)
 *            2 → MOVE_TURN_180
 *            0 → MOVE_DONE (same heading — no turn needed)
 *
 *          Truth table for all 12 heading transitions:
 *            N→E: diff=1 → RIGHT    E→S: diff=1 → RIGHT
 *            S→W: diff=1 → RIGHT    W→N: diff=1 → RIGHT
 *            N→W: diff=3 → LEFT     W→S: diff=3 → LEFT
 *            S→E: diff=3 → LEFT     E→N: diff=3 → LEFT
 *            N→S: diff=2 → 180      S→N: diff=2 → 180
 *            E→W: diff=2 → 180      W→E: diff=2 → 180
 */
MoveType_t path_optimizer_turn_type(uint8_t from_dir, uint8_t to_dir)
{
    if (from_dir >= NUM_DIRS || to_dir >= NUM_DIRS)
    {
        return MOVE_DONE;
    }

    uint8_t diff = (uint8_t)((to_dir - from_dir + (uint8_t)NUM_DIRS)
                              % (uint8_t)NUM_DIRS);

    switch (diff)
    {
        case 1U:  return MOVE_TURN_RIGHT;
        case 3U:  return MOVE_TURN_LEFT;
        case 2U:  return MOVE_TURN_180;
        case 0U:
        default:  return MOVE_DONE;   /* Same direction — no turn */
    }
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Print the optimised path as human-readable text (stub when logging disabled).
 */
void path_optimizer_print(const OptPath_t *path)
{
    (void)path;
}
