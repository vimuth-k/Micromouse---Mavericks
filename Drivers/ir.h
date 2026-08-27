/**
 * @file    ir.h
 * @brief   6-channel IR proximity sensor system — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef IR_H
#define IR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

#define IR_NUM_PAIRS            6U

#define IR_RS      0U   /**< Right Side  */
#define IR_LS      1U   /**< Left  Side  */
#define IR_RF      2U   /**< Right Front */
#define IR_LF      3U   /**< Left  Front */
#define IR_R_ANG   4U   /**< Right Angle */
#define IR_L_ANG   5U   /**< Left  Angle */

typedef struct
{
    uint32_t magic;
    uint16_t ambient[IR_NUM_PAIRS];
    uint16_t wall_90mm[IR_NUM_PAIRS];
    uint16_t threshold[IR_NUM_PAIRS];
    uint16_t threshold_close;
    uint16_t padding;
} IrCalData_t;

typedef struct
{
    uint16_t ambient[IR_NUM_PAIRS];
    uint16_t lit[IR_NUM_PAIRS];
    uint16_t diff[IR_NUM_PAIRS];
    bool     wall_left;
    bool     wall_right;
    bool     wall_front;
    bool     wall_front_close;
    float    front_error;
    float    side_error;
} IrSnapshot_t;

MmResult_t ir_init(void);

void       ir_update(void);
uint16_t   ir_get_diff(uint8_t idx);
uint16_t   ir_get_raw_batt(void);

bool       ir_wall_front(void);
bool       ir_wall_left(void);
bool       ir_wall_right(void);
bool       ir_wall_front_close(void);

float      ir_front_error(void);
float      ir_side_error(void);

void       ir_cal_ambient(void);
void       ir_cal_wall(void);
MmResult_t ir_cal_save(void);
MmResult_t ir_cal_load(void);
const IrCalData_t *ir_cal_get(void);

void       ir_get_snapshot(IrSnapshot_t *snap);

#ifdef __cplusplus
}
#endif

#endif /* IR_H */
