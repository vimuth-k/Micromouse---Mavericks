/**
 * @file    filters.h
 * @brief   Digital signal filters — public API.
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef FILTERS_H
#define FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define MAV_DEFAULT_N       8U
#define MAV_MAX_N           32U

/* -------------------------------------------------------------------------
 * TYPE 1 — FIRST-ORDER IIR LOW-PASS FILTER (LPF)
 * ----------------------------------------------------------------------- */

typedef struct
{
    float alpha;
    float value;
    bool  seeded;
} LPF_t;

void  lpf_init(LPF_t *f, float alpha);
float lpf_update(LPF_t *f, float x);
float lpf_value(const LPF_t *f);
void  lpf_seed(LPF_t *f, float seed);

/* -------------------------------------------------------------------------
 * TYPE 2 — N-TAP MOVING AVERAGE (MAV)
 * ----------------------------------------------------------------------- */

typedef struct
{
    float    buf[MAV_MAX_N];
    uint8_t  n;
    uint8_t  idx;
    float    sum;
    uint8_t  count;
} MAV_t;

void  mav_init(MAV_t *m, uint8_t n);
float mav_update(MAV_t *m, float x);
float mav_value(const MAV_t *m);
void  mav_reset(MAV_t *m);

/* -------------------------------------------------------------------------
 * TYPE 3 — EXPONENTIAL MOVING AVERAGE (EMA)
 * ----------------------------------------------------------------------- */

typedef struct
{
    float alpha;
    float value;
    bool  seeded;
} EMA_t;

void  ema_init(EMA_t *e, float tau_ms);
float ema_update(EMA_t *e, float x);
float ema_value(const EMA_t *e);
void  ema_seed(EMA_t *e, float seed);

/* -------------------------------------------------------------------------
 * TYPE 4 — MEDIAN OF 3
 * ----------------------------------------------------------------------- */

typedef struct
{
    float  buf[3];
    uint8_t idx;
    bool   ready;
} Median3_t;

void  median3_init(Median3_t *m);
float median3_update(Median3_t *m, float x);
float median3_of(float a, float b, float c);

#ifdef __cplusplus
}
#endif

#endif /* FILTERS_H */
