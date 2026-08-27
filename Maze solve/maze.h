/**
 * @file    maze.h
 * @brief   Maze data structures and wall map — public API.
 *
 * @details This module owns the 16×16 maze wall map and the robot's
 *          current position within it.  It provides the data layer that
 *          floodfill.c (algorithm) and explorer.c (navigation) operate on.
 *
 *          RESPONSIBILITY SPLIT
 *          ─────────────────────
 *          maze.c      : wall map storage, position tracking, wall
 *                        detection integration, map initialisation,
 *                        Flash persistence of the discovered map.
 *          floodfill.c : flood-fill algorithm that reads the wall map
 *                        and writes the distance map.
 *          explorer.c  : search run — calls maze.c to update walls,
 *                        calls floodfill.c to recompute distances,
 *                        calls motion.c to execute moves.
 *          speedrun.c  : speed run — reads the completed map and
 *                        executes the optimal path at high speed.
 *
 *          COORDINATE SYSTEM
 *          ──────────────────
 *          Row 0  = North edge  (top    of the maze)
 *          Row 15 = South edge  (bottom of the maze) ← start side
 *          Col 0  = West  edge  (left   of the maze) ← start corner
 *          Col 15 = East  edge  (right  of the maze)
 *
 *          Robot starts at cell (row=15, col=0), facing North (DIR_N).
 *
 *          Visual:
 *            (0,0)────────────────(0,15)
 *              │                    │
 *              │    goal (7-8,7-8)  │
 *              │                    │
 *            (15,0)──────────────(15,15)
 *              ↑ START (15,0) facing North (↑)
 *
 *          WALL ENCODING
 *          ──────────────
 *          Each cell stores a uint8_t wall bitmap:
 *            Bit 0 (WALL_N) = North wall present
 *            Bit 1 (WALL_E) = East  wall present
 *            Bit 2 (WALL_S) = South wall present
 *            Bit 3 (WALL_W) = West  wall present
 *          Bits 4–7 are reserved.
 *
 *          When a wall is added to cell (r, c) on side DIR:
 *            maze.walls[r][c] |= WALL_FROM[DIR]
 *          The same wall is also set on the neighbouring cell (shared wall):
 *            maze.walls[nr][nc] |= WALL_OPP[DIR]
 *          This ensures the map is always consistent — no cell thinks it
 *          has no wall where its neighbour thinks there is one.
 *
 *          MEMORY
 *          ───────
 *          Wall map  : 16×16 × 1 byte = 256 bytes
 *          Visited   : 16×16 × 1 byte = 256 bytes
 *          Total     : 512 bytes of SRAM (fits easily in 128 KB)
 *          The flood-fill distance map (floodfill.c) adds another 256 bytes.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h — MAZE_SIZE, GOAL_xxx, START_xxx, FLASH_MAZE_ADDR,
 *                     FLASH_MAZE_MAGIC
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef MAZE_H
#define MAZE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"
#include "config.h"

/* =========================================================================
 * DIRECTION CONSTANTS
 * ======================================================================= */

#define DIR_N       0U   /**< North — decreasing row index                */
#define DIR_E       1U   /**< East  — increasing col index                */
#define DIR_S       2U   /**< South — increasing row index                */
#define DIR_W       3U   /**< West  — decreasing col index                */
#define DIR_NONE    0xFFU/**< No direction / invalid                       */
#define NUM_DIRS    4U

/** Wall bit for each direction in the cell wall bitmap. */
#define WALL_N      (1U << DIR_N)   /* 0x01 */
#define WALL_E      (1U << DIR_E)   /* 0x02 */
#define WALL_S      (1U << DIR_S)   /* 0x04 */
#define WALL_W      (1U << DIR_W)   /* 0x08 */

/* =========================================================================
 * MAZE DATA STRUCTURE
 * ======================================================================= */

/**
 * @brief  Complete maze state.
 *
 * @details Declared as a global struct (not a local) because it is 512 bytes
 *          and must persist for the entire run session.  The flood-fill
 *          distance map lives in floodfill.c to keep the two modules
 *          cleanly separated.
 */
typedef struct
{
    /** Wall bitmap per cell. walls[row][col] bit N/E/S/W = wall present. */
    uint8_t walls[MAZE_SIZE][MAZE_SIZE];

    /** Visited flag — 1 = robot has been in this cell and mapped its walls. */
    uint8_t visited[MAZE_SIZE][MAZE_SIZE];

    /** Current robot row (0 = North edge, 15 = South edge). */
    uint8_t robot_row;

    /** Current robot column (0 = West edge, 15 = East edge). */
    uint8_t robot_col;

    /**
     * @brief  Current robot heading (DIR_N/E/S/W).
     *
     * @details This is the direction the robot's front face is pointing,
     *          NOT the direction it last moved.  Updated by maze_turn_right(),
     *          maze_turn_left(), maze_turn_180() after motion is complete.
     */
    uint8_t robot_heading;

} MazeState_t;

/** Global maze state — single instance, owned by maze.c. */
extern MazeState_t maze;

/* =========================================================================
 * DIRECTION GEOMETRY TABLES
 * These are declared extern so floodfill.c and explorer.c can use them
 * without duplicating the arrays.
 * ======================================================================= */

/** Row delta for each direction: N=−1, E=0, S=+1, W=0. */
extern const int8_t  MAZE_DR[NUM_DIRS];

/** Col delta for each direction: N=0, E=+1, S=0, W=−1. */
extern const int8_t  MAZE_DC[NUM_DIRS];

/** Wall bit in THIS cell that blocks travel in a given direction. */
extern const uint8_t MAZE_WALL_FROM[NUM_DIRS];

/** Wall bit in the NEIGHBOUR cell on the opposite side of the shared wall. */
extern const uint8_t MAZE_WALL_OPP[NUM_DIRS];

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise maze to the known start state.
 *
 * @details Clears all walls (except outer boundary and known start cell walls),
 *          clears all visited flags, sets robot position to (START_ROW, START_COL)
 *          facing DIR_N.
 *
 *          Known walls set at init:
 *            - All outer boundary walls (16×4 = 64 wall bits).
 *            - Start cell (15,0): South wall (outer), West wall (outer),
 *              plus any cell-specific start walls noted in the rulebook.
 *              MicroMaze 3 specifies the start cell has 3 walls — the
 *              robot enters from the one open side (North, facing inward).
 *
 *          Call once before every run (search or speed) to reset position.
 *          Does NOT erase the wall map — call maze_clear_walls() for that.
 *
 * @return MM_OK always.
 */
MmResult_t maze_init(void);

/**
 * @brief  Clear ALL wall knowledge (except outer boundary).
 *
 * @details Erases the entire wall map and visited array.
 *          Re-sets the outer boundary walls.
 *          Use before a fresh search run when the maze layout has changed
 *          (e.g. re-run after the final maze differs from the qualifier).
 *
 * @return MM_OK always.
 */
MmResult_t maze_clear_walls(void);

/* =========================================================================
 * WALL MAP — READ
 * ======================================================================= */

/**
 * @brief  Return true if a wall exists on the given side of a cell.
 *
 * @param  row  Cell row    [0, MAZE_SIZE−1].
 * @param  col  Cell column [0, MAZE_SIZE−1].
 * @param  dir  Direction   DIR_N/E/S/W.
 * @return bool  true = wall present.
 */
bool maze_has_wall(uint8_t row, uint8_t col, uint8_t dir);

/**
 * @brief  Return true if a neighbour exists in direction dir from (row,col).
 *
 * @details Returns false if the move would go out of bounds OR if a wall
 *          blocks the path.  A true return guarantees (nr, nc) is valid and
 *          the passage is open.
 *
 * @param  row  Source row.
 * @param  col  Source col.
 * @param  dir  Direction to check.
 * @return bool  true = passage open and neighbour is in bounds.
 */
bool maze_can_move(uint8_t row, uint8_t col, uint8_t dir);

/**
 * @brief  Return the wall bitmap for a cell (all 4 walls in one byte).
 *
 * @param  row  Cell row.
 * @param  col  Cell col.
 * @return uint8_t  Wall bitmap (bits 0–3 = N/E/S/W walls).
 */
uint8_t maze_get_walls(uint8_t row, uint8_t col);

/**
 * @brief  Return true if a cell has been visited during the current search.
 *
 * @param  row  Cell row.
 * @param  col  Cell col.
 * @return bool  true = robot has been here and walls have been mapped.
 */
bool maze_is_visited(uint8_t row, uint8_t col);

/**
 * @brief  Return true if (row, col) is within the 4-cell goal area.
 *
 * @details Goal: rows 7–8, cols 7–8 (GOAL_ROW_MIN/MAX, GOAL_COL_MIN/MAX).
 *
 * @param  row  Cell row.
 * @param  col  Cell col.
 * @return bool  true = goal cell.
 */
bool maze_is_goal(uint8_t row, uint8_t col);

/**
 * @brief  Return true if (row, col) is the start cell (15, 0).
 *
 * @param  row  Cell row.
 * @param  col  Cell col.
 * @return bool  true = start cell.
 */
bool maze_is_start(uint8_t row, uint8_t col);

/* =========================================================================
 * WALL MAP — WRITE
 * ======================================================================= */

/**
 * @brief  Set a wall on one side of a cell and propagate to the neighbour.
 *
 * @details Sets bit (dir) in maze.walls[row][col].
 *          Also sets the opposing wall bit in the adjacent cell so the map
 *          is always self-consistent.  Boundary cells at the maze edge have
 *          no neighbour on the outer side — the boundary is pre-set at init.
 *
 * @param  row  Cell row.
 * @param  col  Cell col.
 * @param  dir  Direction of the wall to set (DIR_N/E/S/W).
 */
void maze_set_wall(uint8_t row, uint8_t col, uint8_t dir);

/**
 * @brief  Mark a cell as visited and update wall map from IR sensor readings.
 *
 * @details Called by explorer.c every time the robot enters a new cell.
 *          Reads ir_wall_front(), ir_wall_left(), ir_wall_right() and
 *          converts relative wall detections (front/left/right) to absolute
 *          compass directions using the robot's current heading.
 *          Sets maze.visited[row][col] = 1.
 *
 * @param  row      Cell row to update.
 * @param  col      Cell col to update.
 * @param  heading  Robot heading when entering this cell (DIR_N/E/S/W).
 */
void maze_update_walls_from_sensors(uint8_t row, uint8_t col, uint8_t heading);

/* =========================================================================
 * ROBOT POSITION TRACKING
 * ======================================================================= */

/**
 * @brief  Update robot position after moving one cell in direction dir.
 *
 * @details Computes new (row, col) from current position + MAZE_DR/DC[dir].
 *          Updates maze.robot_row, maze.robot_col, maze.robot_heading.
 *          Heading is set to dir (direction of travel = new facing direction
 *          at the moment of entry into the next cell).
 *
 *          Does NOT check whether the move is valid (no wall check).
 *          Caller must verify maze_can_move() before calling.
 *
 * @param  dir  Direction of travel (DIR_N/E/S/W).
 */
void maze_advance(uint8_t dir);

/**
 * @brief  Update robot heading after a right turn (without moving).
 *
 * @details heading = (heading + 1) % 4
 */
void maze_turn_right(void);

/**
 * @brief  Update robot heading after a left turn (without moving).
 *
 * @details heading = (heading + 3) % 4
 */
void maze_turn_left(void);

/**
 * @brief  Update robot heading after a 180° turn (without moving).
 *
 * @details heading = (heading + 2) % 4
 */
void maze_turn_180(void);

/**
 * @brief  Force robot position to a specific cell and heading.
 *
 * @details Used to reset position to start (15, 0, DIR_N) before a speed
 *          run, and to re-seat position if the robot is manually repositioned.
 *
 * @param  row      New row.
 * @param  col      New col.
 * @param  heading  New heading (DIR_N/E/S/W).
 */
void maze_set_position(uint8_t row, uint8_t col, uint8_t heading);

/* =========================================================================
 * DIRECTION UTILITIES
 * ======================================================================= */

/**
 * @brief  Convert a relative direction (front/left/right/back) to an
 *         absolute compass direction given the robot's heading.
 *
 * @details Relative directions:
 *            0 = front  (same as heading)
 *            1 = right  (heading + 1) % 4
 *            2 = back   (heading + 2) % 4
 *            3 = left   (heading + 3) % 4
 *
 * @param  heading   Current heading (DIR_N/E/S/W).
 * @param  relative  Relative direction (0=front, 1=right, 2=back, 3=left).
 * @return uint8_t   Absolute direction (DIR_N/E/S/W).
 */
uint8_t maze_relative_to_absolute(uint8_t heading, uint8_t relative);

/**
 * @brief  Return the opposite direction of dir.
 *
 * @details N↔S, E↔W: (dir + 2) % 4.
 *
 * @param  dir  Direction (DIR_N/E/S/W).
 * @return uint8_t  Opposite direction.
 */
uint8_t maze_opposite_dir(uint8_t dir);

/**
 * @brief  Return the neighbour cell coordinates in direction dir.
 *
 * @details Adds MAZE_DR[dir] and MAZE_DC[dir] to (row, col).
 *          Does NOT check bounds — caller must verify before use.
 *
 * @param  row     Source row.
 * @param  col     Source col.
 * @param  dir     Direction.
 * @param[out] nr  Neighbour row.
 * @param[out] nc  Neighbour col.
 */
void maze_neighbour(uint8_t row, uint8_t col, uint8_t dir,
                    uint8_t *nr, uint8_t *nc);

/**
 * @brief  Return the heading character for display ('N','E','S','W').
 *
 * @param  dir  Direction (DIR_N/E/S/W).
 * @return char  'N', 'E', 'S', or 'W'. '?' if invalid.
 */
char maze_dir_char(uint8_t dir);

/* =========================================================================
 * MAP STATISTICS
 * ======================================================================= */

/**
 * @brief  Count the number of visited cells.
 *
 * @return uint16_t  Number of cells with visited flag set (0–256).
 */
uint16_t maze_visited_count(void);

/**
 * @brief  Return true when every reachable cell has been visited.
 *
 * @details A cell is considered reachable if flood-fill distance < 255.
 *          This function calls floodfill.c to get the distance map.
 *          Used by explorer.c to decide when to return to start.
 *
 * @return bool  true = full map exploration complete.
 */
bool maze_is_fully_explored(void);

/* =========================================================================
 * FLASH PERSISTENCE
 * ======================================================================= */

/**
 * @brief  Save the wall map and visited array to Flash sector 7.
 *
 * @details Writes the MazeState_t struct (minus robot position, which
 *          is reset before every run) to FLASH_MAZE_ADDR.
 *          Prepends a magic word (FLASH_MAZE_MAGIC) for validity check.
 *          Blocks during Flash erase + write (~100 ms).
 *
 *          Call after a successful search run so the map survives
 *          a power cycle between the search and the speed run.
 *
 * @return MM_OK           Map saved successfully.
 * @return MM_ERR_STORAGE  Flash erase or write failed.
 */
MmResult_t maze_save_to_flash(void);

/**
 * @brief  Load the wall map and visited array from Flash sector 7.
 *
 * @details Validates the magic word at FLASH_MAZE_ADDR.
 *          If valid, copies the saved MazeState_t into maze (walls + visited).
 *          Robot position is NOT loaded — caller sets position separately.
 *
 * @return MM_OK            Valid map loaded.
 * @return MM_ERR_NOT_FOUND Magic invalid — no saved map.
 */
MmResult_t maze_load_from_flash(void);

/* =========================================================================
 * DEBUG OUTPUT
 * ======================================================================= */

/**
 * @brief  Print the 16×16 wall map as ASCII art (stub when logging disabled).
 *
 * @param  flood  Pointer to a 16×16 uint8_t distance array from floodfill.c.
 *                Pass NULL to omit distance values (show walls only).
 */
void maze_print(const uint8_t flood[MAZE_SIZE][MAZE_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_H */
