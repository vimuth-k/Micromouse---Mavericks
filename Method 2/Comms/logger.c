/**
 * @file    logger.c
 * @brief   MicroMaze 3 · USB-CDC printf-style debug logger.
 * @details
 *   TRANSPORT
 *   ─────────────────────────────────────────────────────────────────────
 *   UART1 is NOT an option on this board's pin map — its TX pin (PA9)
 *   is TIM1_CH2 (motor PWM), a hard conflict, not a firmware choice.
 *   There used to be a dual UART/USB-CDC transport here; the UART path
 *   has been removed entirely rather than left in as dead code (see
 *   git history if you need the old version for reference).
 *
 *   The only remaining transport is USB CDC (virtual COM port), still
 *   stubbed out below behind `#if 0` — the Black Pill's USB connector
 *   is hardwired to PA11/PA12, which are L_ANGLE_EM and DIP1 in this
 *   pin map, so this is ALSO currently blocked. To use it: give up one
 *   of those two signals, enable USB_OTG_FS + CDC class in CubeMX, add
 *   usbd_cdc_if.c/.h to the build, and replace the `#if 0` block below
 *   with a real CDC_Transmit_FS() call.
 *
 *   Until a transport is freed up, LOG_ENABLED is 0 in config.h, which
 *   strips every LOG_* call site to nothing at the macro level (see
 *   logger.h) — this file still compiles and links cleanly either way,
 *   it just has nowhere to send bytes yet.
 *
 *   BUFFER
 *   ─────────────────────────────────────────────────────────────────────
 *   One static LOG_BUF_SIZE-byte buffer is reused for every call.
 *   vsnprintf() never overflows it; long lines are truncated cleanly.
 *   This is safe only because every call site is main-loop context
 *   (see the ISR-safety note in logger.h) — there is no reentrancy
 *   protection.
 *
 * @author  VDawn
 * @date    2026
 */
#include "logger.h"
#include "config.h"
#include "error.h"
#include "main.h"     /* HAL_GetTick */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Module state
 * ═══════════════════════════════════════════════════════════════════════ */

/** True once logger_init() has run — gates every transmit. */
static uint8_t s_initialised = 0U;

/** Runtime-adjustable minimum level; messages below this are dropped. */
static LogLevel_t s_active_level = (LogLevel_t)LOG_LEVEL_DEFAULT;

/** Shared formatting buffer, reused by every logger_log()/logger_raw() call. */
static char s_buf[LOG_BUF_SIZE];

/** Fixed-width level tags so columns line up in a terminal. */
static const char *const s_level_tag[LOG_LEVEL_NONE] = {
    "DEBUG",  /* LOG_LEVEL_DEBUG */
    "INFO ",  /* LOG_LEVEL_INFO  */
    "WARN ",  /* LOG_LEVEL_WARN  */
    "ERROR"   /* LOG_LEVEL_ERROR */
};

/* ═══════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Send @p len bytes from @p data over USB CDC. Blocking, bounded
 *         by whatever timeout CDC_Transmit_FS() itself enforces once
 *         it's wired in (see the file banner — this is currently a
 *         no-op stub, there is no active transport yet).
 *
 * @param  data  Bytes to send (need not be NUL-terminated).
 * @param  len   Number of bytes to send.
 */
static void log_transmit(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    /* USB CDC path — enable once USB_OTG_FS + CDC class are added
     * to the build (usbd_cdc_if.c/.h). Left disabled by default so
     * this file compiles standalone without the USB middleware.
     *
     *   extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
     *   (void)CDC_Transmit_FS((uint8_t *)data, len);
     */
#if 0
    extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
    (void)CDC_Transmit_FS((uint8_t *)data, len);
#else
    (void)data;
    (void)len;
#endif
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════ */

void logger_init(void)
{
    s_active_level  = (LogLevel_t)LOG_LEVEL_DEFAULT;
    s_initialised   = 1U;
}

void logger_set_level(LogLevel_t level)
{
    if (level > LOG_LEVEL_NONE)
    {
        return; /* MM_ERR_PARAM-equivalent guard; function is void by design
                  * to match the fire-and-forget style of every other
                  * LOG_* call site. */
    }
    s_active_level = level;
}

LogLevel_t logger_get_level(void)
{
    return s_active_level;
}

void logger_log(LogLevel_t level, const char *fmt, ...)
{
#if LOG_ENABLED
    if ((s_initialised == 0U) || (fmt == NULL))
    {
        return;
    }
    if (level < s_active_level)
    {
        return; /* Filtered out — skip formatting and transmit entirely. */
    }
    if (level >= LOG_LEVEL_NONE)
    {
        return; /* Not a real message level (defensive — should not occur). */
    }

    /* Prefix: "[%8lu ms] [LEVEL] " */
    int32_t prefix_len = snprintf(s_buf, sizeof(s_buf), "[%8lu ms] [%s] ",
                                   (unsigned long)HAL_GetTick(),
                                   s_level_tag[level]);
    if ((prefix_len < 0) || ((size_t)prefix_len >= sizeof(s_buf)))
    {
        return; /* Formatting error or buffer too small for even the prefix. */
    }

    va_list args;
    va_start(args, fmt);
    int32_t msg_len = vsnprintf(s_buf + prefix_len,
                                 sizeof(s_buf) - (size_t)prefix_len,
                                 fmt, args);
    va_end(args);
    if (msg_len < 0)
    {
        return; /* Encoding error from vsnprintf — nothing sane to send. */
    }

    /* Total length actually written, clamped to the buffer (vsnprintf
     * truncates safely on its own; this just computes how much to send). */
    size_t written = (size_t)prefix_len +
        (((size_t)msg_len < (sizeof(s_buf) - (size_t)prefix_len))
             ? (size_t)msg_len
             : (sizeof(s_buf) - (size_t)prefix_len - 1U));

    /* Append CRLF if there is room; otherwise send without it rather
     * than overflow the buffer. */
    if (written + 2U < sizeof(s_buf))
    {
        s_buf[written]     = '\r';
        s_buf[written + 1U] = '\n';
        written += 2U;
    }

    log_transmit((const uint8_t *)s_buf, (uint16_t)written);
#else
    (void)level;
    (void)fmt;
#endif /* LOG_ENABLED */
}

void logger_flush(void)
{
    /* No-op — see the Doxygen comment in logger.h. Every LOG_* call
     * already blocks until its bytes are transmitted, so there is
     * nothing buffered to flush with the current synchronous design. */
}

void logger_raw(const char *fmt, ...)
{
#if LOG_ENABLED
    if ((s_initialised == 0U) || (fmt == NULL))
    {
        return;
    }

    va_list args;
    va_start(args, fmt);
    int32_t len = vsnprintf(s_buf, sizeof(s_buf), fmt, args);
    va_end(args);
    if (len < 0)
    {
        return;
    }

    size_t written = ((size_t)len < sizeof(s_buf))
                          ? (size_t)len
                          : (sizeof(s_buf) - 1U);

    log_transmit((const uint8_t *)s_buf, (uint16_t)written);
#else
    (void)fmt;
#endif /* LOG_ENABLED */
}
