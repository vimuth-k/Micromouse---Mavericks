/**
 * @file    flash_storage.c
 * @brief   MicroMaze 3 · Flash sector-7 storage coordinator — implementation.
 *
 * @author  VDawn
 * @date    2026
 */
#include "flash_storage.h"
#include "config.h"
#include "error.h"
#include "ir.h"
#include "main.h"
#include <string.h>

#define MAZE_DATA_SIZE (4U + (2U * (uint32_t)MAZE_SIZE * MAZE_SIZE))

static uint8_t s_backup_buf[MAZE_DATA_SIZE];

MmResult_t flash_storage_load_calibration(void)
{
    return ir_cal_load();
}

static HAL_StatusTypeDef flash_program_words(uint32_t addr, const uint32_t *data, uint32_t bytes)
{
    uint32_t words = (bytes + 3U) / 4U;
    for (uint32_t i = 0U; i < words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (i * 4U), data[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

MmResult_t flash_storage_prepare_write(FlashRegion_t writing_region)
{
    bool     has_backup = false;
    uint32_t other_addr = 0U;
    uint32_t other_size = 0U;

    if (writing_region == FLASH_REGION_CAL)
    {
        if (*(const uint32_t *)FLASH_MAZE_ADDR == FLASH_MAZE_MAGIC)
        {
            has_backup = true;
            other_addr = FLASH_MAZE_ADDR;
            other_size = MAZE_DATA_SIZE;
        }
    }
    else
    {
        const IrCalData_t *cal = (const IrCalData_t *)FLASH_CAL_ADDR;
        if (cal->magic == FLASH_CAL_MAGIC)
        {
            has_backup = true;
            other_addr = FLASH_CAL_ADDR;
            other_size = sizeof(IrCalData_t);
        }
    }

    if (has_backup)
    {
        (void)memcpy(s_backup_buf, (const void *)other_addr, other_size);
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return MM_ERR_STORAGE;
    }

    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = FLASH_CAL_SECTOR,
        .NbSectors    = 1U,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0U;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        (void)HAL_FLASH_Lock();
        return MM_ERR_STORAGE;
    }

    if (has_backup && flash_program_words(other_addr, (const uint32_t *)s_backup_buf, other_size) != HAL_OK)
    {
        (void)HAL_FLASH_Lock();
        return MM_ERR_STORAGE;
    }

    return MM_OK;
}
