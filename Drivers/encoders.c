/**
 * @file    encoders.c
 * @brief   Quadrature encoder reading — implementation.
 *
 * @author  VDawn
 * @date    2026
 */

#include "encoders.h"
#include "pins.h"
#include "config.h"
#include "main.h"

static volatile int32_t  s_right_overflow = 0;
static volatile uint16_t s_right_last_cnt = 0U;

static int32_t s_left_prev  = 0;
static int32_t s_right_prev = 0;

static int32_t s_left_delta_last  = 0;
static int32_t s_right_delta_last = 0;

static bool s_initialised = false;

static int32_t read_right_count(void)
{
    uint16_t cnt_a;
    uint16_t cnt_b;
    int32_t  ovf;

    cnt_a = ENC_R_CNT;
    ovf   = s_right_overflow;
    cnt_b = ENC_R_CNT;

    if (cnt_b != cnt_a)
    {
        ovf   = s_right_overflow;
        cnt_a = cnt_b;
    }

    int32_t raw = (ovf * 65536) + (int32_t)cnt_a;
    return raw * RIGHT_ENC_POL;
}

MmResult_t encoders_init(void)
{
    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

    ENC_RESET_ALL();

    s_right_overflow   = 0;
    s_right_last_cnt   = 0U;
    s_left_prev        = 0;
    s_right_prev       = 0;
    s_left_delta_last  = 0;
    s_right_delta_last = 0;
    s_initialised      = true;

    return MM_OK;
}

void encoders_reset(void)
{
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);

    ENC_RESET_ALL();
    s_right_overflow   = 0;
    s_right_last_cnt   = 0U;
    s_left_prev        = 0;
    s_right_prev       = 0;
    s_left_delta_last  = 0;
    s_right_delta_last = 0;

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

int32_t enc_left_count(void)
{
    int32_t raw = (int32_t)ENC_L_CNT;
    return raw * LEFT_ENC_POL;
}

int32_t enc_right_count(void)
{
    return read_right_count();
}

int32_t enc_left_delta(void)
{
    int32_t now          = enc_left_count();
    int32_t delta        = now - s_left_prev;
    s_left_prev          = now;
    s_left_delta_last    = delta;
    return delta;
}

int32_t enc_right_delta(void)
{
    int32_t now          = read_right_count();
    int32_t delta        = now - s_right_prev;
    s_right_prev         = now;
    s_right_delta_last   = delta;
    return delta;
}

float enc_left_mm(void)
{
    return (float)enc_left_count() * MM_PER_COUNT;
}

float enc_right_mm(void)
{
    return (float)enc_right_count() * MM_PER_COUNT;
}

float enc_avg_mm(void)
{
    return (enc_left_mm() + enc_right_mm()) * 0.5f;
}

float enc_tracking_error_mm(void)
{
    return enc_left_mm() - enc_right_mm();
}

float enc_left_speed_mmps(void)
{
    if (!s_initialised) { return 0.0f; }
    return (float)s_left_delta_last * MM_PER_COUNT * (float)CTRL_LOOP_HZ;
}

float enc_right_speed_mmps(void)
{
    if (!s_initialised) { return 0.0f; }
    return (float)s_right_delta_last * MM_PER_COUNT * (float)CTRL_LOOP_HZ;
}

void encoders_tim3_ovf_irq(void)
{
    uint16_t cnt = ENC_R_CNT;

    if (cnt < 0x4000U)
    {
        s_right_overflow++;
    }
    else if (cnt > 0xC000U)
    {
        s_right_overflow--;
    }

    s_right_last_cnt = cnt;
}

void encoders_get_state(EncoderState_t *state)
{
    if (state == NULL) { return; }

    state->count_left    = enc_left_count();
    state->count_right   = enc_right_count();
    state->delta_left    = s_left_delta_last;
    state->delta_right   = s_right_delta_last;
    state->dist_left_mm  = enc_left_mm();
    state->dist_right_mm = enc_right_mm();
    state->dist_avg_mm   = enc_avg_mm();
    state->overflow_count = (uint32_t)((s_right_overflow < 0)
                             ? -s_right_overflow
                             :  s_right_overflow);
}
