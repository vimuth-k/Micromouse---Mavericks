/**
 * @file    flash_storage.h
 * @brief   MicroMaze 3 · Flash sector-7 storage coordinator.
 * @details Coordinates shared access to Flash sector 7 between IR calibration
 *          and maze wall map data to prevent sector erases from wiping shared data.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Which sector-7 region a caller is about to program.
 */
typedef enum
{
    FLASH_REGION_CAL = 0U,  /**< IrCalData_t at FLASH_CAL_ADDR.        */
    FLASH_REGION_MAZE       /**< Maze wall map at FLASH_MAZE_ADDR.     */
} FlashRegion_t;

/**
 * @brief  Load IR calibration data from Flash.
 * @return MM_OK on success, MM_ERR_NOT_FOUND if invalid or empty.
 */
MmResult_t flash_storage_load_calibration(void);

/**
 * @brief  Prepare sector 7 for writing to @p writing_region by preserving
 *         the other valid region across the sector erase.
 * @param  writing_region Region to be written (FLASH_REGION_CAL or FLASH_REGION_MAZE).
 * @return MM_OK on success (leaves Flash unlocked for caller writes),
 *         MM_ERR_STORAGE on failure.
 */
MmResult_t flash_storage_prepare_write(FlashRegion_t writing_region);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STORAGE_H */
