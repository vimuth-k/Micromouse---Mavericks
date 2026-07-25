/**
 * @file    logger.h
 * @brief   MicroMaze 3 · UART/USB-CDC printf-style debug logger.
 * @details
 *   WHAT THIS MODULE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   Provides five printf-style macros — LOG_DEBUG, LOG_INFO, LOG_WARN,
 *   LOG_ERROR, LOG_RAW — used throughout the firmware (main.c, scheduler.c,
 *   maze.c, path_optimizer.c, ...) for human-readable debug output over
 *   USART1 (or USB CDC, see the note in logger_init()).
 *
 *   LOG_DEBUG/INFO/WARN/ERROR prepend a millisecond timestamp and a level
 *   tag, and append "\r\n" automatically:
 *
 *       LOG_INFO("Battery: %.2f V", battery_voltage());
 *       -> "[   4213 ms] [INFO ] Battery: 7.42 V\r\n"
 *
 *   LOG_RAW is an unformatted passthrough — no timestamp, no level tag,
 *   no automatic newline — for output that must be exact, such as the
 *   ASCII maze-map dump in maze.c which builds and terminates its own
 *   lines:
 *
 *       LOG_RAW("%s\r\n", row_buf);
 *
 *   RUNTIME LEVEL FILTERING
 *   ─────────────────────────────────────────────────────────────────────
 *   Each call is filtered against a runtime level set with
 *   logger_set_level() (default LOG_LEVEL_DEFAULT from config.h). Calls
 *   below the active level are skipped before formatting, so raising the
 *   level (e.g. to LOG_LEVEL_WARN right before a competition run) also
 *   saves the CPU time that would otherwise go into vsnprintf() and the
 *   blocking UART transmit. LOG_RAW ignores the level filter — it always
 *   transmits when the module is enabled, since it is used for
 *   intentional structured dumps rather than routine trace output.
 *
 *   COMPILE-TIME STRIP
 *   ─────────────────────────────────────────────────────────────────────
 *   Setting LOG_ENABLED to 0 in config.h turns every LOG_* macro into a
 *   no-op at compile time (no code generated, no Flash used, no UART
 *   time spent) — intended for the final competition build.
 *
 *   THREAD / ISR SAFETY
 *   ─────────────────────────────────────────────────────────────────────
 *   NOT safe to call from the TIM5 1 kHz control-loop ISR (or any other
 *   ISR): logger_log()/logger_raw() use a shared static format buffer
 *   and a blocking HAL_UART_Transmit() call, neither of which is
 *   interrupt-safe or fast enough for a 1 kHz deadline. Every existing
 *   call site is in main-loop context (main.c, scheduler.c, maze.c,
 *   path_optimizer.c) — keep it that way.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef LOGGER_H
#define LOGGER_H

#include "config.h"   /* LOG_ENABLED, LOG_LEVEL_DEFAULT, LOG_BUF_SIZE, ... */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Log severity levels, lowest (most verbose) to highest.
 * @details Numeric values match LOG_LEVEL_DEFAULT in config.h. Setting
 *          the active level to LOG_LEVEL_NONE suppresses every LOG_DEBUG
 *          /INFO/WARN/ERROR call (LOG_RAW is unaffected).
 */
typedef enum {
    LOG_LEVEL_DEBUG = 0,  /**< Verbose module-init / state trace. */
    LOG_LEVEL_INFO,       /**< Normal operational messages. */
    LOG_LEVEL_WARN,       /**< Recoverable problems (low battery, etc.). */
    LOG_LEVEL_ERROR,      /**< Failures that abort the current action. */
    LOG_LEVEL_NONE        /**< Suppresses all leveled output. */
} LogLevel_t;

/**
 * @brief  Initialise the logger with a UART handle and bring it online.
 *
 * @details Stores the handle for use by every subsequent LOG_* call and
 *          resets the active level to LOG_LEVEL_DEFAULT (from config.h).
 *          Call once from main.c immediately after uart1_init(), before
 *          any other module that might log during its own init.
 *
 *          USB CDC ALTERNATIVE: USART1_TX is PA9, which is also
 *          TIM1_CH2 (motor PWM) — the two peripherals cannot both own
 *          that pin. If that conflict has not been resolved on your
 *          board, initialise a USB CDC virtual COM port instead (see
 *          uart1_init() in main.c) and pass NULL here; logger.c falls
 *          back to CDC_Transmit_FS() automatically when no UART handle
 *          is supplied. See the LOGGER_USE_USB_CDC note in logger.c.
 *
 * @param  huart  Pointer to an initialised UART_HandleTypeDef (huart1
 *                from main.c), or NULL to use USB CDC instead. Stored
 *                internally — must remain valid for the module's
 *                lifetime.
 */
void logger_init(void *huart);

/**
 * @brief  Change the active runtime log level.
 *
 * @details Messages below this level are dropped before formatting.
 *          LOG_RAW output is never filtered by this setting.
 *
 * @param  level  New minimum level to emit.
 */
void logger_set_level(LogLevel_t level);

/**
 * @brief  Read the current active runtime log level.
 * @return The level most recently set by logger_set_level(), or
 *         LOG_LEVEL_DEFAULT if logger_set_level() has not been called.
 */
LogLevel_t logger_get_level(void);

/**
 * @brief  Format and transmit one leveled log line. Called by the
 *         LOG_DEBUG/INFO/WARN/ERROR macros — not normally called
 *         directly.
 *
 * @details Prepends "[%8lu ms] [LEVEL] " (HAL_GetTick() timestamp) to
 *          the formatted message and appends "\r\n". Silently returns
 *          without formatting or transmitting if @p level is below the
 *          active level, if the module has not been initialised, or if
 *          LOG_ENABLED is 0.
 *
 * @param  level  Severity of this message — compared against the
 *                active level set by logger_set_level().
 * @param  fmt    printf-style format string.
 * @param  ...    Format arguments.
 */
void logger_log(LogLevel_t level, const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

/**
 * @brief  Transmit one line with no timestamp, no level tag, and no
 *         automatic newline. Called by the LOG_RAW macro — not
 *         normally called directly.
 *
 * @details Intended for output whose exact bytes matter, such as the
 *          maze-map ASCII dump in maze.c, where the caller controls
 *          line termination itself. Ignores the active log level —
 *          only LOG_ENABLED and successful logger_init() gate it.
 *
 * @param  fmt  printf-style format string.
 * @param  ...  Format arguments.
 */
void logger_raw(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

/* ═══════════════════════════════════════════════════════════════════════
 * Public macros — use these, not logger_log()/logger_raw() directly.
 * ═══════════════════════════════════════════════════════════════════════ */
#if LOG_ENABLED

/** Verbose trace — module init steps, per-tick debug values. */
#define LOG_DEBUG(fmt, ...)  logger_log(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)

/** Normal operational messages — boot banner, mode selection, results. */
#define LOG_INFO(fmt, ...)   logger_log(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)

/** Recoverable problems — low battery, unexpected-but-handled state. */
#define LOG_WARN(fmt, ...)   logger_log(LOG_LEVEL_WARN,  fmt, ##__VA_ARGS__)

/** Failures — init errors, stall detection, fatal conditions. */
#define LOG_ERROR(fmt, ...)  logger_log(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

/** Raw passthrough — no prefix, no auto newline; caller formats exactly. */
#define LOG_RAW(fmt, ...)    logger_raw(fmt, ##__VA_ARGS__)

#else /* LOG_ENABLED == 0 : strip every call site to nothing */

#define LOG_DEBUG(fmt, ...)  ((void)0)
#define LOG_INFO(fmt, ...)   ((void)0)
#define LOG_WARN(fmt, ...)   ((void)0)
#define LOG_ERROR(fmt, ...)  ((void)0)
#define LOG_RAW(fmt, ...)    ((void)0)

#endif /* LOG_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
