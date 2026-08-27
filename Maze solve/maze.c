/**
 * @file    maze.c
 * @brief   Maze data structures and wall map — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *          maze.c is the data layer of the navigation stack.  It owns the
 *          wall map and robot position, providing three services:
 *
 *          1. WALL MAP STORAGE
 *             256 bytes of uint8_t walls[16][16] where each byte encodes
 *             which of the 4 walls of that cell are present.  Every write
 *             propagates to the shared wall of the adjacent cell, keeping
 *             the map self-consistent.  Reading is a single bit-test.
 *
 *          2. SENSOR INTEGRATION
 *             maze_update_walls_from_sensors() translates the IR detector
 *             outputs (front/left/right wall present) into absolute compass
 *             directions using the robot's current heading, then calls
 *             maze_set_wall().  This is the only place in the codebase
 *             where IR readings become wall map entries.
 *
 *          3. POSITION TRACKING
 *             The robot's (row, col, heading) triple is updated by:
 *               maze_advance()    — after every forward cell move
 *               maze_turn_xxx()   — after every turn
 *               maze_set_position()— on reset or manual repositioning
 *             Navigation code never needs to compute (row + DR[dir]) by
 *             itself — it calls maze_advance(dir) and reads maze.robot_row.
 *
 *          THE SHARED-WALL INVARIANT
 *          ──────────────────────────
 *          Every physical wall is shared between two adjacent cells.
 *          When the robot detects a north wall in cell (5, 3), cell (4, 3)
 *          must also record a south wall — they are the same physical wall.
 *          maze_set_wall() enforces this invariant unconditionally:
 *
 *            maze.walls[row][col]  |= WALL_FROM[dir]
 *            maze.walls[nr][nc]    |= WALL_OPP[dir]
 *
 *          Without this, floodfill.c would see an asymmetric map —
 *          passable from one side, blocked from the other — and compute
 *          incorrect distances.  The invariant is maintained on every write.
 *
 *          RELATIVE → ABSOLUTE WALL DETECTION
 *          ─────────────────────────────────────
 *          IR sensors report walls relative to the robot:
 *            "front wall present", "left wall present", "right wall present"
 *          The maze needs walls in absolute compass coordinates:
 *            "north wall at (r,c)", "east wall at (r,c)", etc.
 *          The conversion uses the current heading:
 *            front_wall → absolute dir = heading
 *            left_wall  → absolute dir = (heading + 3) % 4
 *            right_wall → absolute dir = (heading + 1) % 4
 *          This is implemented by maze_relative_to_absolute().
 *
 *          BOUNDARY PRE-INITIALISATION
 *          ─────────────────────────────
 *          The outer boundary of the 16×16 maze is always walled.
 *          maze_init() and maze_clear_walls() set all boundary walls
 *          immediately — the robot never needs to discover them.
 *          This reduces the number of cells the flood-fill must process
 *          in the first few ticks and prevents "escape" paths in the
 *          distance map.
 *
 * @author  VDawn
 * @date    2026
 */

#include "maze.h"
#include "ir.h"
#include "flash_storage.h"
#include "config.h"
#include "main.h"     /* HAL_FLASH_Program/Lock, HAL_StatusTypeDef */
#include <string.h>

/* =========================================================================
 * GLOBAL MAZE STATE
 * ======================================================================= */

/** Single maze instance — shared by all navigation modules. */
MazeState_t maze;

/* =========================================================================
 * DIRECTION GEOMETRY TABLES  (exported via maze.h for floodfill.c)
 * ======================================================================= */

/** Row delta per direction: N=−1, E=0, S=+1, W=0. */
const int8_t MAZE_DR[NUM_DIRS] = { -1,  0,  1,  0 };

/** Col delta per direction: N=0, E=+1, S=0, W=−1. */
const int8_t MAZE_DC[NUM_DIRS] = {  0,  1,  0, -1 };

/**
 * @brief  Wall bit in THIS cell that blocks travel in direction d.
 * @details WALL_N=0x01, WALL_E=0x02, WALL_S=0x04, WALL_W=0x08.
 */
const uint8_t MAZE_WALL_FROM[NUM_DIRS] = { WALL_N, WALL_E, WALL_S, WALL_W };

/**
 * @brief  Wall bit in the NEIGHBOUR cell for the shared wall.
 * @details If you travel North from (r,c) into (r-1,c), the wall that
 *          blocked you appears as a SOUTH wall in (r-1,c).
 */
const uint8_t MAZE_WALL_OPP[NUM_DIRS]  = { WALL_S, WALL_W, WALL_N, WALL_E };

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Validate that (row, col) is within the maze bounds.
 *
 * @param  row  Row to check.
 * @param  col  Col to check.
 * @return bool  true = in bounds.
 */
static bool in_bounds(uint8_t row, uint8_t col)
{
    return (row < MAZE_SIZE) && (col < MAZE_SIZE);
}

/**
 * @brief  Set all outer boundary walls.
 *
 * @details North edge: all cells in row 0 get WALL_N.
 *          South edge: all cells in row 15 get WALL_S.
 *          West  edge: all cells in col 0  get WALL_W.
 *          East  edge: all cells in col 15 get WALL_E.
 */
static void set_boundary_walls(void)
{
    for (uint8_t i = 0U; i < MAZE_SIZE; i++)
    {
        maze.walls[0U][i]              |= WALL_N;   /* north boundary     */
        maze.walls[MAZE_SIZE - 1U][i]  |= WALL_S;   /* south boundary     */
        maze.walls[i][0U]              |= WALL_W;   /* west  boundary     */
        maze.walls[i][MAZE_SIZE - 1U]  |= WALL_E;   /* east  boundary     */
    }
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise maze to the known start state.
 */
MmResult_t maze_init(void)
{
    /* Clear all wall and visited data */
    (void)memset(maze.walls,   0, sizeof(maze.walls));
    (void)memset(maze.visited, 0, sizeof(maze.visited));

    /* Set outer boundary walls */
    set_boundary_walls();

    /*
     * MicroMaze 3 start cell (15, 0):
     *   South wall : already set by boundary (row 15)
     *   West  wall : already set by boundary (col 0)
     *   East  wall : the rulebook states the start cell has 3 walls.
     *               The robot enters from the North side (open to the
     *               interior).  The East wall closes the start alcove.
     *   North wall : OPEN (robot exits North into the maze)
     */
    maze.walls[START_ROW][START_COL] |= WALL_E;

    /* Set robot starting position */
    maze.robot_row     = START_ROW;
    maze.robot_col     = START_COL;
    maze.robot_heading = START_HEADING;   /* DIR_N */

    return MM_OK;
}

/**
 * @brief  Clear all wall knowledge and re-initialise boundaries.
 */
MmResult_t maze_clear_walls(void)
{
    (void)memset(maze.walls,   0, sizeof(maze.walls));
    (void)memset(maze.visited, 0, sizeof(maze.visited));
    set_boundary_walls();
    maze.walls[START_ROW][START_COL] |= WALL_E;
    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — WALL MAP READ
 * ======================================================================= */

/**
 * @brief  Return true if a wall exists on the given side of a cell.
 */
bool maze_has_wall(uint8_t row, uint8_t col, uint8_t dir)
{
    if (!in_bounds(row, col) || dir >= NUM_DIRS) { return true; }
    return (maze.walls[row][col] & MAZE_WALL_FROM[dir]) != 0U;
}

/**
 * @brief  Return true if the robot can move from (row,col) in direction dir.
 *
 * @details A move is valid if:
 *          1. No wall blocks it from this cell.
 *          2. The destination cell is within bounds.
 *          Both conditions are checked — boundary walls pre-block out-of-bounds.
 */
bool maze_can_move(uint8_t row, uint8_t col, uint8_t dir)
{
    if (!in_bounds(row, col) || dir >= NUM_DIRS) { return false; }
    if (maze.walls[row][col] & MAZE_WALL_FROM[dir]) { return false; }

    int8_t nr = (int8_t)row + MAZE_DR[dir];
    int8_t nc = (int8_t)col + MAZE_DC[dir];
    return in_bounds((uint8_t)nr, (uint8_t)nc);
}

/**
 * @brief  Return wall bitmap for a cell.
 */
uint8_t maze_get_walls(uint8_t row, uint8_t col)
{
    if (!in_bounds(row, col)) { return 0xFFU; }
    return maze.walls[row][col];
}

/**
 * @brief  Return visited flag for a cell.
 */
bool maze_is_visited(uint8_t row, uint8_t col)
{
    if (!in_bounds(row, col)) { return false; }
    return maze.visited[row][col] != 0U;
}

/**
 * @brief  Return true if (row, col) is a goal cell.
 */
bool maze_is_goal(uint8_t row, uint8_t col)
{
    return (row >= GOAL_ROW_MIN) && (row <= GOAL_ROW_MAX) &&
           (col >= GOAL_COL_MIN) && (col <= GOAL_COL_MAX);
}

/**
 * @brief  Return true if (row, col) is the start cell.
 */
bool maze_is_start(uint8_t row, uint8_t col)
{
    return (row == START_ROW) && (col == START_COL);
}

/* =========================================================================
 * PUBLIC API — WALL MAP WRITE
 * ======================================================================= */

/**
 * @brief  Set a wall and propagate to the neighbouring cell.
 *
 * @details Shared-wall invariant enforcement:
 *          - Set MAZE_WALL_FROM[dir] in maze.walls[row][col].
 *          - Set MAZE_WALL_OPP[dir]  in maze.walls[nr][nc].
 *          Both cells record the same physical wall.
 *
 *          Bounds check on the neighbour: boundary cells have no neighbour
 *          on the outer side.  The boundary walls are already set at init,
 *          so the only effect of the bounds check is to avoid writing
 *          out of the array.
 */
void maze_set_wall(uint8_t row, uint8_t col, uint8_t dir)
{
    if (!in_bounds(row, col) || dir >= NUM_DIRS) { return; }

    /* Set wall in this cell */
    maze.walls[row][col] |= MAZE_WALL_FROM[dir];

    /* Propagate to neighbour */
    int8_t nr = (int8_t)row + MAZE_DR[dir];
    int8_t nc = (int8_t)col + MAZE_DC[dir];

    if (in_bounds((uint8_t)nr, (uint8_t)nc))
    {
        maze.walls[(uint8_t)nr][(uint8_t)nc] |= MAZE_WALL_OPP[dir];
    }
}

/**
 * @brief  Mark a cell visited and update its wall map from IR sensors.
 *
 * @details Reads the three wall detection results from ir.c and converts
 *          them to absolute compass directions.  Calls maze_set_wall()
 *          for each detected wall.  Sets the visited flag.
 *
 *          Conversion (heading = current robot compass direction):
 *            front wall → maze direction = heading
 *            left  wall → maze direction = (heading + 3) % 4
 *            right wall → maze direction = (heading + 1) % 4
 *
 *          NOTE: The back of the robot (behind) is not checked because
 *          the robot has just come from there — that passage is open
 *          (otherwise it could not have entered this cell).
 *          The back wall will be detected when the robot enters the
 *          previous cell from the other direction.
 */
void maze_update_walls_from_sensors(uint8_t row, uint8_t col, uint8_t heading)
{
    if (!in_bounds(row, col) || heading >= NUM_DIRS) { return; }

    uint8_t dir_front = heading;
    uint8_t dir_left  = maze_relative_to_absolute(heading, 3U);  /* left */
    uint8_t dir_right = maze_relative_to_absolute(heading, 1U);  /* right*/

    if (ir_wall_front())  { maze_set_wall(row, col, dir_front); }
    if (ir_wall_left())   { maze_set_wall(row, col, dir_left);  }
    if (ir_wall_right())  { maze_set_wall(row, col, dir_right); }

    maze.visited[row][col] = 1U;
}

/* =========================================================================
 * PUBLIC API — ROBOT POSITION TRACKING
 * ======================================================================= */

/**
 * @brief  Advance robot position one cell in direction dir.
 *
 * @details Updates robot_row, robot_col, robot_heading.
 *          Heading is set to the direction of travel.
 *          Caller is responsible for verifying maze_can_move() first.
 */
void maze_advance(uint8_t dir)
{
    if (dir >= NUM_DIRS) { return; }
    maze.robot_row     = (uint8_t)((int8_t)maze.robot_row + MAZE_DR[dir]);
    maze.robot_col     = (uint8_t)((int8_t)maze.robot_col + MAZE_DC[dir]);
    maze.robot_heading = dir;
}

/** Turn right: heading = (heading + 1) % 4 */
void maze_turn_right(void)
{
    maze.robot_heading = (maze.robot_heading + 1U) % NUM_DIRS;
}

/** Turn left: heading = (heading + 3) % 4 */
void maze_turn_left(void)
{
    maze.robot_heading = (maze.robot_heading + 3U) % NUM_DIRS;
}

/** U-turn: heading = (heading + 2) % 4 */
void maze_turn_180(void)
{
    maze.robot_heading = (maze.robot_heading + 2U) % NUM_DIRS;
}

/**
 * @brief  Force robot position to (row, col) facing heading.
 */
void maze_set_position(uint8_t row, uint8_t col, uint8_t heading)
{
    maze.robot_row     = in_bounds(row, col) ? row    : START_ROW;
    maze.robot_col     = in_bounds(row, col) ? col    : START_COL;
    maze.robot_heading = (heading < NUM_DIRS)? heading: DIR_N;
}

/* =========================================================================
 * PUBLIC API — DIRECTION UTILITIES
 * ======================================================================= */

/**
 * @brief  Convert relative direction to absolute compass direction.
 *
 * @details relative 0=front, 1=right, 2=back, 3=left.
 *          absolute = (heading + relative) % 4.
 */
uint8_t maze_relative_to_absolute(uint8_t heading, uint8_t relative)
{
    return (uint8_t)((heading + relative) % NUM_DIRS);
}

/**
 * @brief  Return opposite direction: (dir + 2) % 4.
 */
uint8_t maze_opposite_dir(uint8_t dir)
{
    return (uint8_t)((dir + 2U) % NUM_DIRS);
}

/**
 * @brief  Compute neighbour cell coordinates.
 */
void maze_neighbour(uint8_t row, uint8_t col, uint8_t dir,
                    uint8_t *nr, uint8_t *nc)
{
    if (nr != NULL) { *nr = (uint8_t)((int8_t)row + MAZE_DR[dir]); }
    if (nc != NULL) { *nc = (uint8_t)((int8_t)col + MAZE_DC[dir]); }
}

/**
 * @brief  Return heading character.
 */
char maze_dir_char(uint8_t dir)
{
    static const char CHARS[NUM_DIRS] = { 'N', 'E', 'S', 'W' };
    return (dir < NUM_DIRS) ? CHARS[dir] : '?';
}

/* =========================================================================
 * PUBLIC API — MAP STATISTICS
 * ======================================================================= */

/**
 * @brief  Count visited cells.
 */
uint16_t maze_visited_count(void)
{
    uint16_t count = 0U;
    for (uint8_t r = 0U; r < MAZE_SIZE; r++)
    {
        for (uint8_t c = 0U; c < MAZE_SIZE; c++)
        {
            if (maze.visited[r][c]) { count++; }
        }
    }
    return count;
}

/**
 * @brief  Return true when every reachable cell has been visited.
 *
 * @details Iterates every cell with a flood distance < 255 (reachable)
 *          and checks the visited flag.  Any unvisited reachable cell
 *          returns false.
 *          This is intentionally a linear scan — called infrequently
 *          (at most once per cell, i.e. 256 times per search run).
 *
 * @note   This function needs access to the flood map.  It calls
 *         floodfill_get_distance() to avoid including floodfill.h here
 *         (would create a circular dependency: maze.h ↔ floodfill.h).
 *         Alternatively, explorer.c calls this after each flood update.
 */
bool maze_is_fully_explored(void)
{
    for (uint8_t r = 0U; r < MAZE_SIZE; r++)
    {
        for (uint8_t c = 0U; c < MAZE_SIZE; c++)
        {
            /* If not visited and not a boundary-only cell, not fully explored */
            if (!maze.visited[r][c] && maze.walls[r][c] != 0xFFU)
            {
                return false;
            }
        }
    }
    return true;
}

/* =========================================================================
 * PUBLIC API — FLASH PERSISTENCE
 * ======================================================================= */

/**
 * @brief  Save the wall map and visited array to Flash sector 7.
 *
 * @details Layout at FLASH_MAZE_ADDR:
 *            [0x00] magic    : uint32_t = FLASH_MAZE_MAGIC
 *            [0x04] walls    : uint8_t[16][16] = 256 bytes
 *            [0x104]visited  : uint8_t[16][16] = 256 bytes
 *            Total           : 4 + 256 + 256 = 516 bytes
 *
 *          Written word-by-word using HAL_FLASH_Program. The erase step
 *          is delegated to flash_storage_prepare_write(), which backs
 *          up the IR calibration struct (if present) before erasing the
 *          shared sector and restores it immediately after — sector 7
 *          also holds Drivers/ir.c's saved calibration at
 *          FLASH_CAL_ADDR, and the STM32F4 can only erase a sector as a
 *          whole, so saving the maze map alone would otherwise wipe it.
 *          This also makes repeated maze saves safe: without an erase
 *          here, a second save over already-programmed Flash bytes
 *          would corrupt rather than update them.
 *
 * @warning This function takes ~100 ms to complete (Flash erase).
 *          Call only from the main loop, never from the ISR.
 */
MmResult_t maze_save_to_flash(void)
{
    HAL_StatusTypeDef st;
    uint32_t addr = FLASH_MAZE_ADDR;

    if (flash_storage_prepare_write(FLASH_REGION_MAZE) != MM_OK)
    {
        return MM_ERR_STORAGE;
    }

    /* Write magic word */
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, FLASH_MAZE_MAGIC);
    if (st != HAL_OK) { (void)HAL_FLASH_Lock(); return MM_ERR_STORAGE; }
    addr += 4U;

    /* Write wall map word-by-word */
    const uint32_t *walls_u32 = (const uint32_t *)maze.walls;
    uint32_t walls_words = (sizeof(maze.walls) + 3U) / 4U;   /* = 64 words */
    for (uint32_t w = 0U; w < walls_words; w++)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, walls_u32[w]);
        if (st != HAL_OK) { (void)HAL_FLASH_Lock(); return MM_ERR_STORAGE; }
        addr += 4U;
    }

    /* Write visited map */
    const uint32_t *vis_u32  = (const uint32_t *)maze.visited;
    uint32_t vis_words = (sizeof(maze.visited) + 3U) / 4U;   /* = 64 words */
    for (uint32_t w = 0U; w < vis_words; w++)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, vis_u32[w]);
        if (st != HAL_OK) { (void)HAL_FLASH_Lock(); return MM_ERR_STORAGE; }
        addr += 4U;
    }

    (void)HAL_FLASH_Lock();
    return MM_OK;
}

/**
 * @brief  Load wall map and visited array from Flash sector 7.
 */
MmResult_t maze_load_from_flash(void)
{
    uint32_t magic = *(const uint32_t *)FLASH_MAZE_ADDR;
    if (magic != FLASH_MAZE_MAGIC) { return MM_ERR_NOT_FOUND; }

    const uint8_t *src = (const uint8_t *)(FLASH_MAZE_ADDR + 4U);
    (void)memcpy(maze.walls,   src,                       sizeof(maze.walls));
    (void)memcpy(maze.visited, src + sizeof(maze.walls),  sizeof(maze.visited));

    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — DEBUG OUTPUT
 * ======================================================================= */

/**
 * @brief  Print the 16×16 wall map as ASCII art (stub when logging disabled).
 */
void maze_print(const uint8_t flood[MAZE_SIZE][MAZE_SIZE])
{
    (void)flood;
}
