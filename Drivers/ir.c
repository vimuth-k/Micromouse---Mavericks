/**
 * @file    ir.c
 * @brief   6-channel IR proximity sensor system — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "ir.h"
#include "pins.h"
#include "config.h"
#include "flash_storage.h"
#include "utils.h"
#include "main.h"
#include <string.h>

static uint16_t adc_dma_buf[IR_ADC_BUF_LEN];

static uint16_t s_ambient[IR_NUM_PAIRS] = { 0U };
static uint16_t s_lit[IR_NUM_PAIRS]     = { 0U };
static uint16_t s_diff[IR_NUM_PAIRS]    = { 0U };

static IrCalData_t s_cal;

static bool s_initialised = false;
static volatile bool s_dma_fresh = false;

static void (*const EMIT_ON[IR_NUM_PAIRS])(void) = {
    IR_EMIT_RS_ON,
    IR_EMIT_LS_ON,
    IR_EMIT_RF_ON,
    IR_EMIT_LF_ON,
    IR_EMIT_R_ANG_ON,
    IR_EMIT_L_ANG_ON
};

static void (*const EMIT_OFF[IR_NUM_PAIRS])(void) = {
    IR_EMIT_RS_OFF,
    IR_EMIT_LS_OFF,
    IR_EMIT_RF_OFF,
    IR_EMIT_LF_OFF,
    IR_EMIT_R_ANG_OFF,
    IR_EMIT_L_ANG_OFF
};

static const uint8_t PAIR_ADC_IDX[IR_NUM_PAIRS] = {
    IR_IDX_RS,
    IR_IDX_LS,
    IR_IDX_RF,
    IR_IDX_LF,
    IR_IDX_R_ANG,
    IR_IDX_L_ANG
};

static void install_default_thresholds(void)
{
    s_cal.magic = 0U;

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        s_cal.ambient[i]   = 0U;
        s_cal.wall_90mm[i] = 0U;
    }

    s_cal.threshold[IR_RF]    = IR_THRESH_FRONT;
    s_cal.threshold[IR_LF]    = IR_THRESH_FRONT;
    s_cal.threshold[IR_RS]    = IR_THRESH_SIDE;
    s_cal.threshold[IR_LS]    = IR_THRESH_SIDE;
    s_cal.threshold[IR_R_ANG] = IR_THRESH_DIAG;
    s_cal.threshold[IR_L_ANG] = IR_THRESH_DIAG;

    s_cal.threshold_close = IR_THRESH_FRONT_CLOSE;
}

static void read_pair(uint8_t pair_idx)
{
    uint8_t adc_slot = PAIR_ADC_IDX[pair_idx];

    delay_us(IR_EMITTER_SETTLE_US);
    s_ambient[pair_idx] = adc_dma_buf[adc_slot];

    EMIT_ON[pair_idx]();
    delay_us(IR_EMITTER_SETTLE_US);

    s_lit[pair_idx] = adc_dma_buf[adc_slot];

    EMIT_OFF[pair_idx]();

    if (s_lit[pair_idx] > s_ambient[pair_idx])
    {
        uint16_t raw_diff = s_lit[pair_idx] - s_ambient[pair_idx];
        s_diff[pair_idx]  = (raw_diff < (uint16_t)IR_NOISE_FLOOR)
                            ? 0U : raw_diff;
    }
    else
    {
        s_diff[pair_idx] = 0U;
    }
}

MmResult_t ir_init(void)
{
    IR_ALL_EMITTERS_OFF();

    (void)memset(adc_dma_buf, 0, sizeof(adc_dma_buf));
    (void)memset(s_ambient,   0, sizeof(s_ambient));
    (void)memset(s_lit,       0, sizeof(s_lit));
    (void)memset(s_diff,      0, sizeof(s_diff));

    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc1,
                                                 (uint32_t *)adc_dma_buf,
                                                 IR_ADC_BUF_LEN);
    if (status != HAL_OK)
    {
        return MM_ERR_DRIVER;
    }

    if (ir_cal_load() != MM_OK)
    {
        install_default_thresholds();
    }

    s_initialised = true;
    s_dma_fresh   = false;

    return MM_OK;
}

void ir_update(void)
{
    if (!s_initialised) { return; }

    IR_ALL_EMITTERS_OFF();

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        read_pair(i);
    }

    IR_ALL_EMITTERS_OFF();
    s_dma_fresh = false;
}

uint16_t ir_get_diff(uint8_t idx)
{
    if (idx >= IR_NUM_PAIRS) { return 0U; }
    return s_diff[idx];
}

uint16_t ir_get_raw_batt(void)
{
    return adc_dma_buf[IR_IDX_BATT];
}

bool ir_wall_front(void)
{
    return (s_diff[IR_RF] > s_cal.threshold[IR_RF]) ||
           (s_diff[IR_LF] > s_cal.threshold[IR_LF]);
}

bool ir_wall_left(void)
{
    return s_diff[IR_LS] > s_cal.threshold[IR_LS];
}

bool ir_wall_right(void)
{
    return s_diff[IR_RS] > s_cal.threshold[IR_RS];
}

bool ir_wall_front_close(void)
{
    return (s_diff[IR_RF] > s_cal.threshold_close) &&
           (s_diff[IR_LF] > s_cal.threshold_close);
}

float ir_front_error(void)
{
    return (float)s_diff[IR_LF] - (float)s_diff[IR_RF];
}

float ir_side_error(void)
{
    return (float)s_diff[IR_LS] - (float)s_diff[IR_RS];
}

void ir_cal_ambient(void)
{
    IR_ALL_EMITTERS_OFF();
    delay_us(200U);

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        s_cal.ambient[i] = adc_dma_buf[PAIR_ADC_IDX[i]];
    }
}

void ir_cal_wall(void)
{
    ir_update();

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        s_cal.wall_90mm[i] = s_diff[i];

        uint32_t thresh = ((uint32_t)s_diff[i] * 40U) / 100U;
        s_cal.threshold[i] = (thresh < 60U) ? 60U : (uint16_t)thresh;
    }

    uint32_t front_avg = ((uint32_t)s_cal.wall_90mm[IR_RF]
                        +  (uint32_t)s_cal.wall_90mm[IR_LF]) / 2U;
    uint32_t close = (front_avg * 80U) / 100U;
    s_cal.threshold_close = (close < 80U) ? 80U : (uint16_t)close;
}

MmResult_t ir_cal_save(void)
{
    s_cal.magic   = FLASH_CAL_MAGIC;
    s_cal.padding = 0U;

    HAL_StatusTypeDef status;

    if (flash_storage_prepare_write(FLASH_REGION_CAL) != MM_OK)
    {
        return MM_ERR_STORAGE;
    }

    const uint32_t *src  = (const uint32_t *)&s_cal;
    uint32_t        addr = FLASH_CAL_ADDR;
    uint32_t        words = (sizeof(IrCalData_t) + 3U) / 4U;

    for (uint32_t w = 0U; w < words; w++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[w]);
        if (status != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return MM_ERR_STORAGE;
        }
        addr += 4U;
    }

    (void)HAL_FLASH_Lock();
    return MM_OK;
}

MmResult_t ir_cal_load(void)
{
    const IrCalData_t *flash_data = (const IrCalData_t *)FLASH_CAL_ADDR;

    if (flash_data->magic != FLASH_CAL_MAGIC)
    {
        return MM_ERR_NOT_FOUND;
    }

    (void)memcpy(&s_cal, flash_data, sizeof(IrCalData_t));
    return MM_OK;
}

const IrCalData_t *ir_cal_get(void)
{
    return &s_cal;
}

void ir_get_snapshot(IrSnapshot_t *snap)
{
    if (snap == NULL) { return; }

    for (uint8_t i = 0U; i < IR_NUM_PAIRS; i++)
    {
        snap->ambient[i] = s_ambient[i];
        snap->lit[i]     = s_lit[i];
        snap->diff[i]    = s_diff[i];
    }

    snap->wall_left        = ir_wall_left();
    snap->wall_right       = ir_wall_right();
    snap->wall_front       = ir_wall_front();
    snap->wall_front_close = ir_wall_front_close();
    snap->front_error      = ir_front_error();
    snap->side_error       = ir_side_error();
}
