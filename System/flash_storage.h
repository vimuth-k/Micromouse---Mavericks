/**
 * @file    flash_storage.h
 * @brief   MicroMaze 3 · Flash sector-7 write coordinator.
 * @details
 *   THE PROBLEM THIS MODULE SOLVES
 *   ─────────────────────────────────────────────────────────────────────
 *   Two independent pieces of data share Flash sector 7 (STM32F411,
 *   128 KB, the only sector not needed by the program image):
 *
 *       Offset 0x000  IrCalData_t   (Drivers/ir.c)     — 44 bytes
 *       Offset 0x100  maze wall map (Maze solve/maze.c) — 516 bytes
 *
 *   The STM32F4 can only erase a sector as a whole 128 KB block — there
 *   is no way to erase or reprogram just one region without touching
 *   the other. If ir.c and maze.c each independently unlocked, erased,
 *   and programmed sector 7 for their own data, whichever one saved
 *   last would silently wipe out the other's data.
 *
 *   flash_storage_prepare_write() is the single choke point both
 *   modules go through before writing: it backs up whichever region it
 *   is NOT about to overwrite into RAM, erases the sector, and
 *   immediately restores the backed-up region — so by the time it
 *   returns, sector 7 contains a freshly-erased target region and an
 *   intact copy of the other one. The caller (ir_cal_save() or
 *   maze_save_to_flash()) then programs its own region exactly as
 *   before.
 *
 *   OWNERSHIP OF FLASH LOCK/UNLOCK
 *   ─────────────────────────────────────────────────────────────────────
 *   flash_storage_prepare_write() calls HAL_FLASH_Unlock() and
 *   deliberately leaves Flash unlocked on success, since the caller
 *   still needs to program its own region immediately afterward. The
 *   caller is responsible for calling HAL_FLASH_Lock() itself once its
 *   own writes are complete (both ir_cal_save() and
 *   maze_save_to_flash() already do this — only their erase step
 *   changes). On failure, flash_storage_prepare_write() locks Flash
 *   again before returning so a failed prepare never leaves the part
 *   unlocked.
 *
 *   READS ARE UNAFFECTED
 *   ─────────────────────────────────────────────────────────────────────
 *   ir_cal_load() and maze_load_from_flash() read via a direct
 *   memory-mapped pointer — reads never need erase/coordination, so
 *   they are untouched by this module. flash_storage_load_calibration()
 *   exists only because main.c's boot sequence calls it by that name;
 *   it is a one-line passthrough to ir_cal_load().
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
 * @details Thin passthrough to ir_cal_load() — kept as its own entry
 *          point because main.c's boot sequence calls it by this name.
 *          Safe to call at any time; reads are memory-mapped and need
 *          no erase coordination.
 * @return MM_OK           Valid calibration found and copied into RAM.
 * @return MM_ERR_NOT_FOUND No valid calibration in Flash (magic mismatch)
 *                          — caller should fall back to defaults.
 */
MmResult_t flash_storage_load_calibration(void);

/**
 * @brief  Prepare sector 7 for a write to @p writing_region.
 *
 * @details Backs up the OTHER region (if it currently holds valid data,
 *          checked via its magic word) into an internal RAM buffer,
 *          unlocks Flash, erases sector 7, and immediately restores the
 *          backed-up region. Leaves Flash unlocked on success so the
 *          caller can program @p writing_region next — the caller must
 *          call HAL_FLASH_Lock() itself when its own write is done.
 *
 *          Call this in place of a module's own HAL_FLASH_Unlock() +
 *          HAL_FLASHEx_Erase() sequence; everything after that (the
 *          word-by-word HAL_FLASH_Program() loop and the final
 *          HAL_FLASH_Lock()) stays exactly as it already is in
 *          ir_cal_save() / maze_save_to_flash().
 *
 * @param  writing_region  Which region the caller is about to program.
 *                          The other region is the one preserved.
 *
 * @return MM_OK          Sector erased and the other region restored
 *                        (or was empty — nothing to restore). Flash is
 *                        left unlocked; caller must lock it when done.
 * @return MM_ERR_STORAGE Unlock, erase, or restore-write failed. Flash
 *                        is re-locked before returning in this case.
 */
MmResult_t flash_storage_prepare_write(FlashRegion_t writing_region);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STORAGE_H */
