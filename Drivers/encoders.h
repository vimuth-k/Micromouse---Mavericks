/**
 * @file    encoders.h
 * @brief   Quadrature encoder reading — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

typedef struct
{
    int32_t  count_left;
    int32_t  count_right;
    int32_t  delta_left;
    int32_t  delta_right;
    float    dist_left_mm;
    float    dist_right_mm;
    float    dist_avg_mm;
    uint32_t overflow_count;
} EncoderState_t;

MmResult_t encoders_init(void);
void       encoders_reset(void);

int32_t    enc_left_count(void);
int32_t    enc_right_count(void);

int32_t    enc_left_delta(void);
int32_t    enc_right_delta(void);

float      enc_left_mm(void);
float      enc_right_mm(void);
float      enc_avg_mm(void);
float      enc_tracking_error_mm(void);

float      enc_left_speed_mmps(void);
float      enc_right_speed_mmps(void);

void       encoders_tim3_ovf_irq(void);
void       encoders_get_state(EncoderState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ENCODERS_H */
