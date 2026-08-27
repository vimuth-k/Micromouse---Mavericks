/**
 * @file    flash_storage.c
 * @brief   MicroMaze 3 · Flash sector-7 write coordinator — implementation.
 * @details See flash_storage.h for the full rationale. This file knows
 *          the byte layout of both regions (IrCalData_t for
 *          FLASH_REGION_CAL, magic+walls+visited for FLASH_REGION_MAZE)
 *          only well enough to back them up and restore them verbatim —
 *          it does not interpret or validate the contents beyond the
 *          magic-word check, that remains ir.c's and maze.c's job.
 *
 * @author  VDawn
 * @date    2026
 */
#include "flash_storage.h"
#include "config.h"   /* FLASH_CAL_ADDR, FLASH_MAZE_ADDR, FLASH_CAL_SECTOR, ... */
#include "error.h"
#include "ir.h"       /* IrCalData_t, ir_cal_load(), FLASH_CAL_MAGIC check   */
#include "main.h"     /* HAL_FLASH_Unlock/Lock, HAL_FLASHEx_Erase, ...       */
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Region sizing
 * ═══════════════════════════════════════════════════════════════════════ */

/** Bytes in the maze region: magic(4) + walls(MAZE_SIZE²) + visited(MAZE_SIZE²).
 *  Matches the layout maze.c's maze_save_to_flash()/maze_load_from_flash()
 *  already use — kept in sync with MAZE_SIZE from config.h rather than
 *  hardcoded, since it is the larger of the two regions and therefore
 *  sizes the shared backup buffer below. */
#define FLASH_MAZE_REGION_BYTES \
    (4U + ((uint32_t)MAZE_SIZE * MAZE_SIZE) + ((uint32_t)MAZE_SIZE * MAZE_SIZE))

/** Shared backup buffer, sized for the larger of the two regions (maze). */
#define FLASH_BACKUP_BUF_BYTES  FLASH_MAZE_REGION_BYTES

_Static_assert(sizeof(IrCalData_t) <= FLASH_BACKUP_BUF_BYTES,
    "flash_storage.c backup buffer too small for IrCalData_t");

/* ═══════════════════════════════════════════════════════════════════════
 * Module state
 * ═══════════════════════════════════════════════════════════════════════ */

/** Scratch buffer for whichever region is being preserved across an
 *  erase. Only one region is ever backed up at a time — sized for the
 *  larger of the two so it fits both. */
static uint8_t s_backup_buf[FLASH_BACKUP_BUF_BYTES];

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

MmResult_t flash_storage_load_calibration(void)
{
    return ir_cal_load();
}

MmResult_t flash_storage_prepare_write(FlashRegion_t writing_region)
{
    bool     other_valid = false;
    uint32_t other_addr  = 0U;
    uint32_t other_bytes = 0U;

    /* ── Back up whichever region is NOT about to be overwritten ──────── */
    if (writing_region == FLASH_REGION_CAL)
    {
        uint32_t magic = *(const uint32_t *)FLASH_MAZE_ADDR;
        if (magic == FLASH_MAZE_MAGIC)
        {
            other_valid = true;
            other_addr  = FLASH_MAZE_ADDR;
            other_bytes = FLASH_MAZE_REGION_BYTES;
        }
    }
    else /* FLASH_REGION_MAZE */
    {
        const IrCalData_t *cal = (const IrCalData_t *)FLASH_CAL_ADDR;
        if (cal->magic == FLASH_CAL_MAGIC)
        {
            other_valid = true;
            other_addr  = FLASH_CAL_ADDR;
            other_bytes = (uint32_t)sizeof(IrCalData_t);
        }
    }

    if (other_valid)
    {
        (void)memcpy(s_backup_buf, (const void *)other_addr, other_bytes);
    }

    /* ── Unlock, erase the whole sector ────────────────────────────────── */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return MM_ERR_STORAGE;
    }

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = FLASH_CAL_SECTOR,
        .NbSectors    = 1U,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0U;
    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK)
    {
        (void)HAL_FLASH_Lock();
        return MM_ERR_STORAGE;
    }

    /* ── Restore the preserved region immediately, before the caller
     *    programs its own — so a failure partway through the caller's
     *    own write still leaves the other region intact. ─────────────── */
    if (other_valid)
    {
        const uint32_t *src   = (const uint32_t *)s_backup_buf;
        uint32_t         words = (other_bytes + 3U) / 4U;
        uint32_t         addr  = other_addr;

        for (uint32_t w = 0U; w < words; w++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[w]) != HAL_OK)
            {
                (void)HAL_FLASH_Lock();
                return MM_ERR_STORAGE;
            }
            addr += 4U;
        }
    }

    /* Flash intentionally left unlocked — the caller programs its own
     * region next and is responsible for HAL_FLASH_Lock() when done. */
    return MM_OK;
}
