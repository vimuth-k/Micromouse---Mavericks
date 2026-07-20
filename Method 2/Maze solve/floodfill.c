/**
 * @file    floodfill.c
 * @brief   Flood-fill distance map — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          Computes the shortest-path distance from every cell to the
 *          goal (or start) using Breadth-First Search (BFS).  This
 *          distance map is the core of the micromouse maze-solving strategy:
 *          the robot always moves to the adjacent cell with the lowest
 *          distance, which by the properties of BFS is always a step
 *          along the shortest path.
 *
 *          THE BFS ALGORITHM
 *          ──────────────────
 *          BFS is the correct algorithm here because:
 *          - All edges (cell-to-cell passages) have equal weight (1 step).
 *          - BFS on an unweighted graph finds shortest paths in O(V+E) time.
 *          - On a 16×16 maze: V=256 cells, E≤512 passages → < 0.1 ms.
 *          - Dijkstra or A* would also work but are slower for equal weights.
 *
 *          BFS EXECUTION TRACE (simplified 4×4 example):
 *
 *            Initial state: all distances = 255
 *            Goal: (1,1) and (1,2) and (2,1) and (2,2)
 *            Queue: [ (1,1), (1,2), (2,1), (2,2) ]  all at distance 0
 *
 *            Iteration 1: dequeue (1,1), distance=0
 *              Neighbours: (0,1) no wall? → set dist=1, enqueue
 *                          (1,0) no wall? → set dist=1, enqueue
 *                          (2,1) already dist=0, skip
 *                          (1,2) already dist=0, skip
 *            Iteration 2: dequeue (1,2), distance=0
 *              Neighbours: (0,2) → dist=1, enqueue
 *                          (1,3) no wall? → dist=1, enqueue
 *                          ...
 *            ...continues until all reachable cells have distances...
 *
 *          QUEUE IMPLEMENTATION
 *          ─────────────────────
 *          Uses a statically allocated circular FIFO queue.
 *          Maximum queue size = MAZE_SIZE² = 256 entries.
 *          Each entry is a 16-bit value encoding (row, col) as:
 *            entry = row × MAZE_SIZE + col
 *          This allows the entire queue to fit in 512 bytes (uint16_t[256]).
 *          The queue never needs more than MAZE_SIZE² entries because each
 *          cell is enqueued at most once (once its distance is set, it's
 *          never updated again — BFS property on unweighted graphs).
 *
 *          WHY FULL RECOMPUTE (NOT INCREMENTAL)
 *          ──────────────────────────────────────
 *          When a new wall is discovered during search, some cells may now
 *          have a longer minimum path (the short route is blocked) but
 *          distances are never reduced by adding walls — only increased.
 *          An incremental update would need to propagate increased distances
 *          "uphill" from the blocked passage, which requires a priority
 *          queue or repeated passes.
 *          Full BFS recompute from scratch is simpler and takes < 0.1 ms
 *          on a 16×16 maze — faster than the incremental bookkeeping.
 *          This approach scales well up to 32×32 mazes (< 0.4 ms).
 *
 *          PATH EXTRACTION
 *          ────────────────
 *          floodfill_extract_path() follows the gradient downhill from
 *          start to goal: at each cell, move to the open neighbour with
 *          the lowest distance.  This is guaranteed to be shortest because:
 *          - BFS distances are shortest-path distances.
 *          - Greedy descent on shortest-path distances finds the shortest path.
 *          The extracted path is stored as a direction sequence used by
 *          speedrun.c to execute the optimal path without re-running
 *          flood-fill at every cell.
 *
 * @author  VDawn
 * @date    2026
 */

#include "floodfill.h"
#include "maze.h"
#include "config.h"
#include <string.h>

/* =========================================================================
 * PRIVATE — DISTANCE MAP
 * ======================================================================= */

/**
 * @brief  The 16×16 flood-fill distance map.
 *
 * @details dist[row][col] = minimum steps to reach the seeded goal(s).
 *          FLOOD_UNREACHABLE (255) = not yet reached or no path exists.
 *          0 = seed cell (goal in FLOOD_TO_GOAL, start in FLOOD_TO_START).
 */
static uint8_t s_dist[MAZE_SIZE][MAZE_SIZE];

/* =========================================================================
 * PRIVATE — BFS QUEUE
 * ======================================================================= */

/**
 * @brief  Static BFS queue — stores cell indices as (row × SIZE + col).
 *
 * @details Maximum entries needed = MAZE_SIZE² = 256.
 *          Using uint8_t for row and col packed into uint16_t.
 *          head and tail are indices into q_buf, both modulo QUEUE_SIZE.
 *          Queue is empty when head == tail.
 */
#define QUEUE_SIZE  (MAZE_SIZE * MAZE_SIZE)   /* 256 */

static uint16_t s_queue[QUEUE_SIZE];
static uint16_t s_q_head;
static uint16_t s_q_tail;

/**
 * @brief  Enqueue a cell (row, col) into the BFS FIFO.
 */
static inline void q_push(uint8_t row, uint8_t col)
{
    s_queue[s_q_tail] = (uint16_t)((uint16_t)row * MAZE_SIZE + col);
    s_q_tail = (uint16_t)((s_q_tail + 1U) % QUEUE_SIZE);
}

/**
 * @brief  Dequeue the next cell, writing row and col to output pointers.
 */
static inline void q_pop(uint8_t *row, uint8_t *col)
{
    uint16_t entry = s_queue[s_q_head];
    s_q_head = (uint16_t)((s_q_head + 1U) % QUEUE_SIZE);
    *row = (uint8_t)(entry / MAZE_SIZE);
    *col = (uint8_t)(entry % MAZE_SIZE);
}

/**
 * @brief  Return true when the queue is empty.
 */
static inline bool q_empty(void)
{
    return s_q_head == s_q_tail;
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the flood-fill module.
 *
 * @details Fills s_dist with FLOOD_UNREACHABLE and resets the queue.
 */
MmResult_t floodfill_init(void)
{
    (void)memset(s_dist, FLOOD_UNREACHABLE, sizeof(s_dist));
    s_q_head = 0U;
    s_q_tail = 0U;
    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — CORE ALGORITHM
 * ======================================================================= */

/**
 * @brief  Run the complete BFS flood-fill.
 *
 * @details Steps:
 *          1. Reset all distances to FLOOD_UNREACHABLE.
 *          2. Seed the queue with goal cells (FLOOD_TO_GOAL) or
 *             start cell (FLOOD_TO_START) at distance 0.
 *          3. BFS expansion: for each cell dequeued, check all 4
 *             directions. If open (maze_can_move) and neighbour
 *             distance is still FLOOD_UNREACHABLE, set it to
 *             current_distance + 1 and enqueue it.
 *          4. Terminate when queue is empty.
 *
 *          After this function returns, s_dist contains the shortest-path
 *          distance from every reachable cell to the seeded goal(s).
 */
void floodfill_run(FloodMode_t mode)
{
    /* ── 1. Reset distance map ────────────────────────────────────── */
    (void)memset(s_dist, FLOOD_UNREACHABLE, sizeof(s_dist));

    /* ── 2. Reset queue ───────────────────────────────────────────── */
    s_q_head = 0U;
    s_q_tail = 0U;

    /* ── 3. Seed the queue ────────────────────────────────────────── */
    if (mode == FLOOD_TO_GOAL)
    {
        /* Seed all 4 goal cells — the 2×2 block at rows 7–8, cols 7–8 */
        for (uint8_t r = GOAL_ROW_MIN; r <= GOAL_ROW_MAX; r++)
        {
            for (uint8_t c = GOAL_COL_MIN; c <= GOAL_COL_MAX; c++)
            {
                s_dist[r][c] = FLOOD_GOAL_DIST;
                q_push(r, c);
            }
        }
    }
    else   /* FLOOD_TO_START */
    {
        /* Seed the single start cell */
        s_dist[START_ROW][START_COL] = FLOOD_GOAL_DIST;
        q_push(START_ROW, START_COL);
    }

    /* ── 4. BFS expansion ─────────────────────────────────────────── */
    while (!q_empty())
    {
        uint8_t r, c;
        q_pop(&r, &c);

        uint8_t cur_dist = s_dist[r][c];

        /* Examine all 4 directions */
        for (uint8_t dir = 0U; dir < NUM_DIRS; dir++)
        {
            /* Skip if wall blocks this direction */
            if (!maze_can_move(r, c, dir)) { continue; }

            /* Compute neighbour coordinates */
            uint8_t nr = (uint8_t)((int8_t)r + MAZE_DR[dir]);
            uint8_t nc = (uint8_t)((int8_t)c + MAZE_DC[dir]);

            /* Only update if we found a shorter path (first visit = always) */
            if (s_dist[nr][nc] == FLOOD_UNREACHABLE)
            {
                s_dist[nr][nc] = (uint8_t)(cur_dist + 1U);
                q_push(nr, nc);
            }
        }
    }
}

/* =========================================================================
 * PUBLIC API — NAVIGATION
 * ======================================================================= */

/**
 * @brief  Return the best direction to move from (row, col).
 *
 * @details Scans all 4 directions.  For each open direction (no wall,
 *          in bounds), reads the neighbour's flood distance.  Returns
 *          the direction with the minimum distance.
 *
 *          TIE-BREAKING:
 *          When two directions have equal minimum distance, the first one
 *          found wins.  The scan order is N → E → S → W (DIR_N=0 first),
 *          so ties are broken in favour of North, then East, etc.
 *          Consistent tie-breaking gives reproducible paths on symmetric
 *          sections of the maze.
 *
 *          DEAD-END HANDLING:
 *          If all 4 directions are blocked or lead to FLOOD_UNREACHABLE,
 *          returns DIR_NONE.  explorer.c must handle this — it indicates
 *          either a completely walled cell or a disconnected map region.
 *          In a valid MicroMaze 3 competition maze this should never occur
 *          during normal operation (all cells are reachable).
 */
uint8_t floodfill_best_direction(uint8_t row, uint8_t col)
{
    uint8_t best_dir  = DIR_NONE;
    uint8_t best_dist = FLOOD_UNREACHABLE;

    for (uint8_t dir = 0U; dir < NUM_DIRS; dir++)
    {
        /* Must be a passable direction */
        if (!maze_can_move(row, col, dir)) { continue; }

        uint8_t nr = (uint8_t)((int8_t)row + MAZE_DR[dir]);
        uint8_t nc = (uint8_t)((int8_t)col + MAZE_DC[dir]);

        uint8_t nd = s_dist[nr][nc];

        /* Strictly less-than for tie-breaking: first direction wins */
        if (nd < best_dist)
        {
            best_dist = nd;
            best_dir  = dir;
        }
    }

    return best_dir;
}

/**
 * @brief  Return the flood distance of a specific cell.
 */
uint8_t floodfill_get_distance(uint8_t row, uint8_t col)
{
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) { return FLOOD_UNREACHABLE; }
    return s_dist[row][col];
}

/**
 * @brief  Return pointer to the full 16×16 distance map.
 */
const uint8_t *floodfill_get_map(void)
{
    return &s_dist[0][0];
}

/* =========================================================================
 * PUBLIC API — PATH EXTRACTION
 * ======================================================================= */

/**
 * @brief  Extract the optimal path from start to goal.
 *
 * @details Precondition: floodfill_run(FLOOD_TO_GOAL) must have been
 *          called and the map must be current.
 *
 *          Algorithm:
 *          Starting at (START_ROW, START_COL), at each cell call
 *          floodfill_best_direction() to get the next step, record
 *          the direction in buf, and advance the position.
 *          Terminate when a goal cell is reached (distance == 0)
 *          or buf_len is exhausted (path too long — shouldn't happen
 *          in a 16×16 maze where max path length ≤ 256).
 *
 *          The extracted path is used by speedrun.c to pre-load the
 *          entire optimal sequence before the speed run begins, so the
 *          run executes the sequence at full speed without flood-fill
 *          recomputation at each cell.
 *
 * @param[out] buf      Caller-allocated direction array.
 * @param[in]  buf_len  Maximum number of directions to write.
 * @return uint8_t      Number of directions written (= path length).
 *                      0 = no path found or start is already at goal.
 */
uint8_t floodfill_extract_path(uint8_t *buf, uint8_t buf_len)
{
    if (buf == NULL || buf_len == 0U) { return 0U; }

    uint8_t r    = START_ROW;
    uint8_t c    = START_COL;
    uint8_t step = 0U;

    /* If already at goal, nothing to do */
    if (maze_is_goal(r, c)) { return 0U; }

    while (step < buf_len)
    {
        uint8_t dir = floodfill_best_direction(r, c);

        if (dir == DIR_NONE)
        {
            /* No valid move — maze is unsolvable from here */
            break;
        }

        buf[step++] = dir;

        /* Advance position */
        r = (uint8_t)((int8_t)r + MAZE_DR[dir]);
        c = (uint8_t)((int8_t)c + MAZE_DC[dir]);

        /* Stop when goal is reached */
        if (maze_is_goal(r, c)) { break; }
    }

    return step;
}

/**
 * @brief  Count the steps in the optimal path (flood distance of start).
 */
uint8_t floodfill_path_length(void)
{
    return s_dist[START_ROW][START_COL];
}

/* =========================================================================
 * PUBLIC API — DIAGNOSTICS
 * ======================================================================= */

/**
 * @brief  Count reachable cells (distance < FLOOD_UNREACHABLE).
 */
uint16_t floodfill_reachable_count(void)
{
    uint16_t count = 0U;
    for (uint8_t r = 0U; r < MAZE_SIZE; r++)
    {
        for (uint8_t c = 0U; c < MAZE_SIZE; c++)
        {
            if (s_dist[r][c] < FLOOD_UNREACHABLE) { count++; }
        }
    }
    return count;
}

/**
 * @brief  Return true when a path to the goal exists from the start.
 */
bool floodfill_goal_reachable(void)
{
    return s_dist[START_ROW][START_COL] < FLOOD_UNREACHABLE;
}
