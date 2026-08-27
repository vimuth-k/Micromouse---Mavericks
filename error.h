/**
 * @file    error.h
 * @brief   MicroMaze 3 · Shared result/error type for every module.
 * @details
 *   WHAT THIS FILE DOES
 *   ─────────────────────────────────────────────────────────────────────
 *   Defines a single enum, `MmResult_t`, that every driver, control,
 *   navigation, and system module returns from its fallible functions
 *   (init routines, Flash read/write, calibration, etc.). It is the
 *   common vocabulary the whole firmware uses to report success/failure
 *   without exceptions or errno-style globals.
 *
 *   There is no error.c — this header is a pure type definition with
 *   zero behaviour, so nothing needs to be compiled or linked for it.
 *   Callers just switch/compare on the returned value, e.g.:
 *
 *       if (ir_init() != MM_OK) {
 *           /* handle error */
 *       }
 *
 *   CODE MEANINGS
 *   ─────────────────────────────────────────────────────────────────────
 *   MM_OK             Operation completed successfully.
 *   MM_ERR_DRIVER      Underlying peripheral/hardware call failed
 *                       (I2C NACK, ADC/DMA didn't start, timeout waiting
 *                       on a busy flag, etc.).
 *   MM_ERR_STORAGE     Flash erase or write failed, or a Flash read did
 *                       not verify (used by ir.c calibration save/load
 *                       and maze.c map save/load).
 *   MM_ERR_NOT_FOUND   Requested data does not exist yet — e.g. no valid
 *                       calibration/maze map in Flash (magic word check
 *                       failed). Not necessarily an error the caller
 *                       should treat as fatal; often means "fall back to
 *                       defaults".
 *   MM_ERR_PARAM       A function argument was out of range or otherwise
 *                       invalid (NULL pointer, index out of bounds, etc.).
 *   MM_ERR_OVERFLOW     A fixed-size buffer, queue, or counter would have
 *                       exceeded its capacity (e.g. path optimizer run
 *                       array, floodfill FIFO).
 *   MM_ERR_GENERAL     Catch-all for failures that don't fit the other
 *                       categories.
 *
 *   USAGE NOTES
 *   ─────────────────────────────────────────────────────────────────────
 *   - MM_OK is guaranteed to be 0, so `if (fn() != MM_OK)` and
 *     `if (fn())` are both valid ways to check for failure.
 *   - This header has no dependency on config.h, pins.h, or the HAL —
 *     any module can include it first, before anything else.
 *
 * @author  VDawn
 * @date    2026
 */
#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Common return/result type used across the entire firmware.
 *
 * MM_OK is always 0 so that `if (fn() != MM_OK)` and the shorter
 * `if (fn())` are both correct ways to test for failure.
 */
typedef enum {
    MM_OK = 0,          /**< Success — no error. */
    MM_ERR_DRIVER,      /**< Hardware/peripheral call failed. */
    MM_ERR_STORAGE,     /**< Flash erase/write failed or did not verify. */
    MM_ERR_NOT_FOUND,   /**< Requested data not present (e.g. no cal in Flash). */
    MM_ERR_PARAM,       /**< Invalid argument (NULL pointer, out of range). */
    MM_ERR_OVERFLOW,    /**< Fixed-size buffer/queue/counter would overflow. */
    MM_ERR_GENERAL      /**< Unclassified failure. */
} MmResult_t;

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
