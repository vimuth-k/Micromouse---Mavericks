/**
 * @file    oled.h
 * @brief   SSD1306 128×64 OLED display driver — public API.
 *
 * @details Manages a SSD1306 OLED display over I2C1 for real-time
 *          robot status feedback during development and competition.
 *
 *          HARDWARE
 *          ────────
 *          Display : SSD1306 monochrome 128×64 pixels.
 *          Interface: I2C1 at 400 kHz (PB8 SCL, PB9 SDA).
 *          Address  : 0x3C (SA0 pin tied to GND).
 *          Power    : 3.3 V from the MCU rail.
 *
 *          FRAMEBUFFER ARCHITECTURE
 *          ─────────────────────────
 *          All drawing operates on an 8-page × 128-column RAM buffer
 *          (8 × 128 = 1024 bytes) held entirely in MCU SRAM.
 *          Each byte in the buffer represents 8 vertically-stacked pixels
 *          in one column.  Bit 0 = top pixel, bit 7 = bottom pixel.
 *
 *          The display is never written to directly — all functions write
 *          to the buffer, and oled_flush() transfers the buffer to the
 *          SSD1306 over I2C in one sequential burst per page (8 bursts
 *          total, each 129 bytes: 1 control byte + 128 data bytes).
 *
 *          DISPLAY LAYOUT (128×64 pixels, 8 pages of 8 px)
 *          ─────────────────────────────────────────────────
 *          Using a 6×8 font (6 px wide, 8 px tall = 1 page tall):
 *            128 / 6 = 21 characters per row.
 *            64  / 8 =  8 rows on screen.
 *
 *          Page/row mapping:
 *            Page 0 (row 0): Status line  — mode name or run state
 *            Page 1 (row 1): Sensor line  — FL FR LS RS values
 *            Page 2 (row 2): Motion line  — speed L/R mm/s
 *            Page 3 (row 3): Encoder line — position mm
 *            Page 4 (row 4): Gyro line    — yaw angle
 *            Page 5 (row 5): Maze line    — row col heading
 *            Page 6 (row 6): Battery line — voltage and percent
 *            Page 7 (row 7): Error line   — last error code
 *
 *          NON-BLOCKING REFRESH
 *          ─────────────────────
 *          oled_flush() blocks for approximately 8 × (129 byte I2C burst)
 *          at 400 kHz ≈ 2.6 ms.  It must NOT be called from the 1 kHz
 *          ISR.  Call it from the main loop at OLED_REFRESH_MS intervals
 *          using a soft timer in scheduler.c.
 *
 *          DEPENDENCIES
 *          ─────────────
 *          config.h — OLED_I2C_ADDR, OLED_WIDTH_PX, OLED_HEIGHT_PX,
 *                     OLED_PAGES, OLED_REFRESH_MS,
 *                     FW_VERSION_MAJOR/MINOR/PATCH, all MODE_xxx,
 *                     SPD_RUN1/2/3
 *          main.h   — hi2c1 extern
 *
 * @author  VDawn
 * @date    2026
 */

#ifndef OLED_H
#define OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "error.h"
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * TYPES
 * ======================================================================= */

/**
 * @brief  SSD1306 handle — holds I2C handle and framebuffer pointer.
 *
 * @details Passed to oled_init().  Allows the module to use whatever
 *          I2C handle main.c provides without hardcoding hi2c1.
 */
typedef struct
{
    void    *hi2c;         /**< Pointer to I2C_HandleTypeDef (cast to void*) */
    uint8_t  i2c_addr_8;   /**< 8-bit I2C address = 7-bit addr << 1         */
} OledHandle_t;

/* =========================================================================
 * SSD1306 REGISTER / COMMAND CONSTANTS
 * Used internally but exposed here so calibration or test code can
 * issue raw commands if needed via oled_send_cmd().
 * ======================================================================= */

#define OLED_CMD_DISPLAY_OFF        0xAEU  /**< Sleep mode                 */
#define OLED_CMD_DISPLAY_ON         0xAFU  /**< Normal display on          */
#define OLED_CMD_SET_CONTRAST       0x81U  /**< Followed by contrast value */
#define OLED_CMD_ENTIRE_DISP_OFF    0xA4U  /**< Output follows RAM         */
#define OLED_CMD_NORMAL_DISP        0xA6U  /**< Non-inverted               */
#define OLED_CMD_INVERT_DISP        0xA7U  /**< Inverted                   */
#define OLED_CMD_SET_MEM_ADDR_MODE  0x20U  /**< Followed by mode byte      */
#define OLED_CMD_ADDR_MODE_HORIZ    0x00U  /**< Horizontal addressing      */
#define OLED_CMD_SET_PAGE_ADDR      0xB0U  /**< OR with page 0-7           */
#define OLED_CMD_SET_COL_LOW        0x00U  /**< Set col low nibble to 0    */
#define OLED_CMD_SET_COL_HIGH       0x10U  /**< Set col high nibble to 0   */
#define OLED_CMD_SET_START_LINE     0x40U  /**< Start line = 0             */
#define OLED_CMD_SEG_REMAP          0xA1U  /**< Column 127 mapped to SEG0  */
#define OLED_CMD_COM_SCAN_DEC       0xC8U  /**< Scan from COM[N-1] to COM0 */
#define OLED_CMD_SET_COM_PINS       0xDAU  /**< COM pins config, +0x12     */
#define OLED_CMD_SET_MULTIPLEX      0xA8U  /**< Multiplex ratio, +0x3F     */
#define OLED_CMD_SET_DISP_OFFSET    0xD3U  /**< Display offset, +0x00      */
#define OLED_CMD_SET_DISP_CLK       0xD5U  /**< Clock div, +0x80           */
#define OLED_CMD_SET_PRECHARGE      0xD9U  /**< Pre-charge, +0xF1          */
#define OLED_CMD_SET_VCOM           0xDBU  /**< VCOMH deselect, +0x30      */
#define OLED_CMD_CHARGE_PUMP        0x8DU  /**< Charge pump, +0x14 = ON    */

/* =========================================================================
 * LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the SSD1306 and clear the display.
 *
 * @details Sends the full SSD1306 initialisation command sequence over
 *          I2C, clears the framebuffer to all zeros, and flushes once
 *          so the physical display shows a blank screen.
 *          Call once during system bring-up after hi2c1 is ready.
 *
 * @param  hi2c  Pointer to the I2C_HandleTypeDef to use (hi2c1 from main.c).
 *               Stored internally — must remain valid for the lifetime of
 *               the module.
 *
 * @return MM_OK          Initialisation and first flush succeeded.
 * @return MM_ERR_DRIVER  I2C communication failed — check PB8/PB9 and
 *                        4.7 kΩ pull-up resistors.
 */
MmResult_t oled_init(void *hi2c);

/**
 * @brief  Clear the framebuffer to all zeros (all pixels off).
 *
 * @details Does NOT flush to the display — call oled_flush() after
 *          to make the clear visible.  Use oled_clear_flush() if you
 *          want both in one call.
 */
void oled_clear(void);

/**
 * @brief  Clear the framebuffer and immediately flush to the display.
 *
 * @details Convenience wrapper: oled_clear() followed by oled_flush().
 *          Blocks for ~2.6 ms (I2C transfer time).
 */
void oled_clear_flush(void);

/**
 * @brief  Transfer the entire framebuffer to the SSD1306 over I2C.
 *
 * @details Writes 8 pages sequentially.  Each page is 129 bytes:
 *          1 control byte (0x40 = data mode) + 128 pixel bytes.
 *          Total I2C bytes = 8 × 129 = 1032 bytes.
 *          At 400 kHz (fast mode): ≈ 2.6 ms.
 *
 * @warning Must NOT be called from the 1 kHz control loop ISR.
 *          Call from the main loop scheduler at OLED_REFRESH_MS intervals.
 *
 * @return MM_OK          All pages written successfully.
 * @return MM_ERR_DRIVER  I2C error during transfer.
 */
MmResult_t oled_flush(void);

/* =========================================================================
 * PRIMITIVE DRAWING
 * ======================================================================= */

/**
 * @brief  Set or clear a single pixel in the framebuffer.
 *
 * @param  x    Column (0–127, left to right).
 * @param  y    Row    (0– 63, top  to bottom).
 * @param  on   true = pixel on (white), false = pixel off (black).
 */
void oled_pixel(uint8_t x, uint8_t y, bool on);

/**
 * @brief  Draw a horizontal line in the framebuffer.
 *
 * @param  x     Start column.
 * @param  y     Row (0–63).
 * @param  len   Length in pixels.
 * @param  on    true = on, false = off.
 */
void oled_hline(uint8_t x, uint8_t y, uint8_t len, bool on);

/**
 * @brief  Draw a filled rectangle in the framebuffer.
 *
 * @param  x     Left column.
 * @param  y     Top row.
 * @param  w     Width in pixels.
 * @param  h     Height in pixels.
 * @param  on    true = fill on, false = fill off (erase).
 */
void oled_rect_fill(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);

/* =========================================================================
 * TEXT RENDERING
 * ======================================================================= */

/**
 * @brief  Draw a single ASCII character at a pixel position.
 *
 * @details Uses the built-in 6×8 font (characters 0x20–0x7E).
 *          Characters outside this range are rendered as a space.
 *          Font is 6 pixels wide: 5 glyph columns + 1 spacer column.
 *          Font is 8 pixels tall: exactly one SSD1306 page.
 *
 * @param  x     Left pixel column of the character (0–122).
 * @param  page  Display page / row (0–7).  Each page = 8 pixel rows.
 * @param  ch    ASCII character to draw.
 */
void oled_char(uint8_t x, uint8_t page, char ch);

/**
 * @brief  Draw a null-terminated string starting at a pixel position.
 *
 * @details Characters are drawn left-to-right with 6-pixel pitch.
 *          String is clipped at the right edge of the display (column 127).
 *          Does not wrap to the next line.
 *
 * @param  x     Left pixel column (0–127).
 * @param  page  Display page / row (0–7).
 * @param  str   Null-terminated ASCII string.
 */
void oled_str(uint8_t x, uint8_t page, const char *str);

/**
 * @brief  Printf-style formatted string rendered to the display.
 *
 * @details Formats into an internal 22-character buffer (21 visible chars
 *          + null terminator) then calls oled_str().
 *          Output is silently truncated at 21 characters.
 *
 * @param  x     Left pixel column.
 * @param  page  Display page (0–7).
 * @param  fmt   printf-style format string.
 * @param  ...   Variadic arguments.
 */
void oled_printf(uint8_t x, uint8_t page, const char *fmt, ...);

/**
 * @brief  Clear a single page (row) to blank, then write a string.
 *
 * @details Convenience function: erases the page first so previous
 *          content does not bleed through, then draws the string.
 *          Commonly used to update a single status line without
 *          clearing the entire display.
 *
 * @param  page  Display page (0–7).
 * @param  str   Null-terminated string (max 21 chars visible).
 */
void oled_line(uint8_t page, const char *str);

/**
 * @brief  Printf-style formatted line — clears the page first.
 *
 * @param  page  Display page (0–7).
 * @param  fmt   printf-style format string.
 * @param  ...   Variadic arguments.
 */
void oled_linef(uint8_t page, const char *fmt, ...);

/* =========================================================================
 * PREDEFINED STATUS SCREENS
 * Each function populates the ENTIRE framebuffer with a specific layout
 * then calls oled_flush() internally.  They block for ~2.6 ms.
 * ======================================================================= */

/**
 * @brief  Boot splash screen.
 *
 * @details Displays firmware version, robot name, and "INIT..." message.
 *          Called once during modules_init() before any hardware is tested.
 *          Layout:
 *            Row 0: "MicroMaze 3"
 *            Row 1: "STM32F411  v1.0.0"
 *            Row 2: blank
 *            Row 3: "INITIALISING..."
 *            Row 4-7: blank
 */
void oled_show_boot(void);

/**
 * @brief  Boot-complete screen.
 *
 * @details Replaces the boot splash once all modules pass initialisation.
 *          Layout:
 *            Row 0: "READY"
 *            Row 1: "Batt: X.XX V  XX%"
 *            Row 2: "Gyro: calibrated"
 *            Row 3: "IR: loaded" or "IR: defaults"
 *            Row 4: "Mode X: <name>"
 *            Row 5-7: blank
 *
 * @param  mode         DIP switch mode number (0–15).
 * @param  batt_v       Battery voltage in volts.
 * @param  cal_loaded   true if calibration was loaded from Flash.
 */
void oled_show_boot_complete(uint8_t mode, float batt_v, bool cal_loaded);

/**
 * @brief  Active mode indicator screen.
 *
 * @details Shows which mode is running with a human-readable name.
 *          Layout:
 *            Row 0: "MODE X"
 *            Row 1: "<mode name>"
 *            Row 2: "Press BTN to run"
 *            Row 3-7: blank
 *
 * @param  mode  DIP switch mode number (0–15).
 */
void oled_show_mode(uint8_t mode);

/**
 * @brief  Live sensor monitor screen (used in MODE_MONITOR).
 *
 * @details Refreshed every OLED_REFRESH_MS from the scheduler.
 *          Layout:
 *            Row 0: "SENSORS"
 *            Row 1: "FL:XXXX FR:XXXX"
 *            Row 2: "LS:XXXX RS:XXXX"
 *            Row 3: "LA:XXXX RA:XXXX"
 *            Row 4: "W: [F][L][R]"    (wall flags)
 *            Row 5: "FErr:XXXXX"
 *            Row 6: "SErr:XXXXX"
 *            Row 7: blank
 *
 * @param  diff         Array of 6 differential ADC values (IR_RS…IR_L_ANG).
 * @param  wall_f       Front wall flag.
 * @param  wall_l       Left  wall flag.
 * @param  wall_r       Right wall flag.
 * @param  front_err    Front balance error (ir_front_error()).
 * @param  side_err     Side  balance error (ir_side_error()).
 */
void oled_show_sensors(const uint16_t diff[6],
                       bool wall_f, bool wall_l, bool wall_r,
                       float front_err, float side_err);

/**
 * @brief  Live motion status screen (used during runs).
 *
 * @details Layout:
 *            Row 0: "RUNNING  Xm Xs"   (elapsed time)
 *            Row 1: "L:XXXXXX R:XXXXXX mm/s"
 *            Row 2: "Pos: XXXXXX mm"
 *            Row 3: "Yaw: XX.XX deg"
 *            Row 4: "Err: XXXXX mm"    (tracking error)
 *            Row 5: "Maze: RR CC H"    (row, col, heading char)
 *            Row 6: "Batt: X.XX V"
 *            Row 7: blank
 *
 * @param  spd_l_mmps   Left  wheel speed mm/s.
 * @param  spd_r_mmps   Right wheel speed mm/s.
 * @param  pos_mm       Robot centre position mm (from encoders).
 * @param  yaw_deg      Current gyro heading degrees.
 * @param  track_err_mm Encoder tracking error mm.
 * @param  maze_row     Current maze row.
 * @param  maze_col     Current maze column.
 * @param  heading      Heading 0=N 1=E 2=S 3=W.
 * @param  elapsed_ms   Elapsed time since run start (ms).
 * @param  batt_v       Battery voltage V.
 */
void oled_show_motion(float    spd_l_mmps,
                      float    spd_r_mmps,
                      float    pos_mm,
                      float    yaw_deg,
                      float    track_err_mm,
                      uint8_t  maze_row,
                      uint8_t  maze_col,
                      uint8_t  heading,
                      uint32_t elapsed_ms,
                      float    batt_v);

/**
 * @brief  Gyro debug screen (MODE_GYRO_DEBUG).
 *
 * @details Layout:
 *            Row 0: "GYRO DEBUG"
 *            Row 1: "Yaw:  XXXXX.X deg"
 *            Row 2: "Rate: XXXXX.X d/s"
 *            Row 3: "Offset: X.XXXX"
 *            Row 4-7: blank
 *
 * @param  yaw_deg     Integrated yaw angle degrees.
 * @param  rate_dps    Raw angular rate deg/s.
 * @param  offset_dps  Calibrated zero-rate offset deg/s.
 */
void oled_show_gyro(float yaw_deg, float rate_dps, float offset_dps);

/**
 * @brief  Calibration progress screen (MODE_IR_CALIBRATE).
 *
 * @details Layout:
 *            Row 0: "IR CALIBRATE"
 *            Row 1: "<step description>"
 *            Row 2: "Press BTN to cont"
 *            Row 3-5: threshold values for all 6 pairs
 *            Row 6: "Save? BTN confirm"
 *            Row 7: blank
 *
 * @param  step    Calibration step (1 = ambient, 2 = wall, 3 = saved).
 * @param  thresh  Array of 6 computed thresholds (may be NULL for step 1).
 */
void oled_show_cal(uint8_t step, const uint16_t thresh[6]);

/**
 * @brief  Battery status screen (MODE_BATTERY_CHECK).
 *
 * @details Layout:
 *            Row 0: "BATTERY"
 *            Row 1: "Voltage: X.XX V"
 *            Row 2: "Level:   XX %"
 *            Row 3: <status string>    "OK" / "LOW" / "CRITICAL"
 *            Row 4-7: visual bar graph  (filled proportional to percent)
 *
 * @param  volts    Battery voltage V.
 * @param  percent  Battery level 0–100 %.
 */
void oled_show_battery(float volts, uint8_t percent);

/**
 * @brief  Maze position and flood-fill value display.
 *
 * @details Used by MODE_SEARCH_RUN and MODE_SPEED_RUN_x.
 *          Layout:
 *            Row 0: "MAZE  R:XX C:XX"
 *            Row 1: "Heading: <N/E/S/W>"
 *            Row 2: "Flood:  XXX"
 *            Row 3: "Walls: F L R"     (presence flags)
 *            Row 4-7: mini 4×4 map of nearby cells (optional)
 *
 * @param  row      Current maze row.
 * @param  col      Current maze column.
 * @param  heading  0=N 1=E 2=S 3=W.
 * @param  flood    Flood-fill distance to goal.
 * @param  wall_f   Front wall present.
 * @param  wall_l   Left  wall present.
 * @param  wall_r   Right wall present.
 */
void oled_show_maze(uint8_t row,    uint8_t col,
                    uint8_t heading,uint8_t flood,
                    bool    wall_f, bool    wall_l, bool wall_r);

/**
 * @brief  Two-line message screen — generic status or prompt.
 *
 * @details Clears the display and shows two centred lines.
 *          Used for "KEEP STILL", "GOAL!", "ERROR", etc.
 *
 * @param  line1  First  line (max 21 chars, auto-centred).
 * @param  line2  Second line (max 21 chars, auto-centred). NULL = blank.
 */
void oled_show_message(const char *line1, const char *line2);

/**
 * @brief  Error screen — large error message with code.
 *
 * @details Clears display and shows:
 *            Row 0: "!! ERROR !!"
 *            Row 1: "<message>"
 *            Row 2: "Code: <code>"
 *            Row 3: "Reset to recover"
 *
 * @param  message  Short error description (max 21 chars).
 * @param  code     Numeric error code.
 */
void oled_show_error(const char *message, int32_t code);

/* =========================================================================
 * LOW-LEVEL ACCESS
 * ======================================================================= */

/**
 * @brief  Send a single command byte to the SSD1306.
 *
 * @details Sends [0x00, cmd] over I2C.  0x00 = control byte
 *          indicating the following byte is a command (Co=0, D/C#=0).
 *          Exposed for test code and calibration screens that need
 *          to set contrast or toggle inversion.
 *
 * @param  cmd  SSD1306 command byte (use OLED_CMD_* constants).
 * @return MM_OK / MM_ERR_DRIVER.
 */
MmResult_t oled_send_cmd(uint8_t cmd);

/**
 * @brief  Return a pointer to the raw framebuffer (for diagnostics).
 *
 * @details Returns a pointer to the 1024-byte framebuffer.
 *          Indexed as buf[page][col].  Read-only use only.
 *
 * @return const uint8_t*  Pointer to framebuffer byte 0 (page 0, col 0).
 */
const uint8_t *oled_get_framebuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
