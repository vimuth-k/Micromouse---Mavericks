/**
 * @file    filters.c
 * @brief   Digital signal filters — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "filters.h"
#include "config.h"
#include <string.h>
#include <math.h>

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

static inline void swap_if_gt(float *a, float *b)
{
    if (*a > *b)
    {
        float tmp = *a;
        *a = *b;
        *b = tmp;
    }
}

/* =========================================================================
 * TYPE 1 — IIR LOW-PASS FILTER (LPF)
 * ======================================================================= */

void lpf_init(LPF_t *f, float alpha)
{
    if (f == NULL) { return; }
    f->alpha  = (alpha < 0.0f) ? 0.0f : (alpha > 1.0f) ? 1.0f : alpha;
    f->value  = 0.0f;
    f->seeded = false;
}

float lpf_update(LPF_t *f, float x)
{
    if (f == NULL) { return x; }
    if (!f->seeded)
    {
        f->value  = x;
        f->seeded = true;
        return x;
    }
    f->value = (f->alpha * x) + ((1.0f - f->alpha) * f->value);
    return f->value;
}

float lpf_value(const LPF_t *f)
{
    return (f != NULL) ? f->value : 0.0f;
}

void lpf_seed(LPF_t *f, float seed)
{
    if (f == NULL) { return; }
    f->value  = seed;
    f->seeded = true;
}

/* =========================================================================
 * TYPE 2 — N-TAP MOVING AVERAGE (MAV)
 * ======================================================================= */

void mav_init(MAV_t *m, uint8_t n)
{
    if (m == NULL) { return; }
    m->n     = (n < 1U) ? 1U : (n > MAV_MAX_N) ? (uint8_t)MAV_MAX_N : n;
    m->idx   = 0U;
    m->sum   = 0.0f;
    m->count = 0U;
    (void)memset(m->buf, 0, sizeof(m->buf));
}

float mav_update(MAV_t *m, float x)
{
    if (m == NULL) { return x; }

    m->sum -= m->buf[m->idx];
    m->buf[m->idx] = x;
    m->sum        += x;

    m->idx = (uint8_t)((m->idx + 1U) % m->n);

    if (m->count < m->n) { m->count++; }

    return m->sum / (float)m->count;
}

float mav_value(const MAV_t *m)
{
    if (m == NULL || m->count == 0U) { return 0.0f; }
    return m->sum / (float)m->count;
}

void mav_reset(MAV_t *m)
{
    if (m == NULL) { return; }
    m->idx   = 0U;
    m->sum   = 0.0f;
    m->count = 0U;
    (void)memset(m->buf, 0, sizeof(m->buf));
}

/* =========================================================================
 * TYPE 3 — EXPONENTIAL MOVING AVERAGE (EMA)
 * ======================================================================= */

void ema_init(EMA_t *e, float tau_ms)
{
    if (e == NULL) { return; }
    float dt_s = 1.0f / (float)CTRL_LOOP_HZ;
    float tau_s = (tau_ms > 0.0f) ? (tau_ms / 1000.0f) : 0.001f;
    e->alpha  = 1.0f - expf(-dt_s / tau_s);
    e->value  = 0.0f;
    e->seeded = false;
}

float ema_update(EMA_t *e, float x)
{
    if (e == NULL) { return x; }
    if (!e->seeded)
    {
        e->value  = x;
        e->seeded = true;
        return x;
    }
    e->value = (e->alpha * x) + ((1.0f - e->alpha) * e->value);
    return e->value;
}

float ema_value(const EMA_t *e)
{
    return (e != NULL) ? e->value : 0.0f;
}

void ema_seed(EMA_t *e, float seed)
{
    if (e == NULL) { return; }
    e->value  = seed;
    e->seeded = true;
}

/* =========================================================================
 * TYPE 4 — MEDIAN OF 3
 * ======================================================================= */

void median3_init(Median3_t *m)
{
    if (m == NULL) { return; }
    m->buf[0] = 0.0f;
    m->buf[1] = 0.0f;
    m->buf[2] = 0.0f;
    m->idx    = 0U;
    m->ready  = false;
}

float median3_update(Median3_t *m, float x)
{
    if (m == NULL) { return x; }

    m->buf[m->idx] = x;
    m->idx = (uint8_t)((m->idx + 1U) % 3U);

    if (!m->ready)
    {
        if (m->idx == 0U) { m->ready = true; }
        else              { return x; }
    }

    float t[3] = { m->buf[0], m->buf[1], m->buf[2] };

    swap_if_gt(&t[0], &t[1]);
    swap_if_gt(&t[0], &t[2]);
    swap_if_gt(&t[1], &t[2]);

    return t[1];
}

float median3_of(float a, float b, float c)
{
    swap_if_gt(&a, &b);
    swap_if_gt(&a, &c);
    swap_if_gt(&b, &c);
    return b;
}
