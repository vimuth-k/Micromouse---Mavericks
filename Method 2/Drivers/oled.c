/**
 * @file    oled.c
 * @brief   SSD1306 128×64 OLED display driver — implementation.
 *
 * @details WHAT THIS FILE DOES
 *          ─────────────────────
 *
 *          1. SSD1306 INITIALISATION SEQUENCE
 *             Sends the mandatory 25-command startup sequence over I2C to
 *             configure the SSD1306 controller: clock divider, multiplex
 *             ratio, COM pin layout, charge pump, contrast, addressing mode.
 *             This sequence is required on every power cycle — the SSD1306
 *             has no persistent configuration.
 *
 *          2. FRAMEBUFFER (1024 bytes in MCU SRAM)
 *             s_fb[page][col]: 8 pages × 128 columns.
 *             Every draw function writes here — never to I2C directly.
 *             oled_flush() transfers the buffer in 8 sequential I2C bursts.
 *             This decouples drawing speed from I2C bus speed and allows
 *             partial updates without waiting for the bus.
 *
 *          3. 6×8 PIXEL FONT (printable ASCII 0x20–0x7E)
 *             95 characters × 5 bytes per glyph = 475 bytes of Flash.
 *             Each glyph is 5 bytes wide and 8 bits tall.
 *             oled_char() writes the 5 glyph bytes plus 1 spacer byte
 *             (6 bytes total = 6-pixel character pitch, 21 chars/line).
 *             Font data is stored in Flash (const array, not RAM).
 *
 *          4. TEXT LAYER (oled_str, oled_printf, oled_line, oled_linef)
 *             Built on top of oled_char().  oled_linef() is the workhorse:
 *             formats a printf string into a 22-byte stack buffer, clears
 *             the target page, then calls oled_str().  Safe: no dynamic
 *             allocation, stack buffer bounded at compile time.
 *
 *          5. STATUS SCREENS
 *             High-level functions called by modes.c and scheduler.c to
 *             show contextually appropriate information.  Each function
 *             clears the buffer, writes all 8 rows, then calls oled_flush().
 *             The scheduler calls oled_show_motion() or oled_show_sensors()
 *             every OLED_REFRESH_MS (100 ms) from the main loop.
 *
 *          6. BATTERY BAR GRAPH
 *             oled_show_battery() draws a 128×16 pixel filled bar whose
 *             width is proportional to battery percentage.  Implemented
 *             using oled_rect_fill() which writes bitmasks directly into
 *             the framebuffer pages.
 *
 *          I2C PROTOCOL DETAILS
 *          ─────────────────────
 *          SSD1306 expects two types of write:
 *            Command: [slave_addr_W] [0x00] [cmd_byte]
 *            Data:    [slave_addr_W] [0x40] [data...up to 128 bytes]
 *          The 0x00 and 0x40 are the SSD1306 "control byte":
 *            Bit 7 (Co):  0 = stream of bytes follows, 1 = single byte.
 *            Bit 6 (D/C): 0 = command, 1 = data.
 *          We always use Co=0 (streaming) so page flushes send all 128
 *          column bytes in one HAL_I2C_Master_Transmit call.
 *
 * @note    Compiled only when OLED_ENABLED = 1 in config.h.
 *
 * @author  VDawn
 * @date    2026
 */

#include "oled.h"

#if OLED_ENABLED

#include "config.h"
#include "main.h"
#include "error.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* =========================================================================
 * PRIVATE — I2C HANDLE
 * ======================================================================= */

/** Pointer to the I2C handle provided by oled_init(). */
static I2C_HandleTypeDef *s_hi2c = NULL;

/** 8-bit I2C write address (7-bit addr << 1). */
#define OLED_ADDR_W   ((uint16_t)((OLED_I2C_ADDR << 1U) & 0xFEU))

/** I2C timeout for each transfer (ms). */
#define I2C_TIMEOUT_MS  10U

/* =========================================================================
 * PRIVATE — FRAMEBUFFER
 * ======================================================================= */

/**
 * @brief  Framebuffer: 8 pages × 128 columns.
 *
 * @details s_fb[page][col].
 *          Page 0 = topmost 8 pixel rows.
 *          Page 7 = bottommost 8 pixel rows.
 *          Within each byte: bit 0 = top pixel of that page row,
 *                            bit 7 = bottom pixel of that page row.
 */
static uint8_t s_fb[OLED_PAGES][OLED_WIDTH_PX];

/* =========================================================================
 * PRIVATE — 6×8 FONT  (ASCII 0x20 – 0x7E, 5 bytes per glyph)
 *
 * Each entry is 5 bytes representing columns 0–4 of the glyph.
 * Bit 0 of each byte = topmost pixel row, bit 7 = bottommost.
 * Column 5 is always a spacer (0x00), added by oled_char().
 * Source: classic Arduino SSD1306 font, public domain.
 * ======================================================================= */
static const uint8_t FONT6x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 20 space  */
    {0x00,0x00,0x5F,0x00,0x00}, /* 21 !      */
    {0x00,0x07,0x00,0x07,0x00}, /* 22 "      */
    {0x14,0x7F,0x14,0x7F,0x14}, /* 23 #      */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* 24 $      */
    {0x23,0x13,0x08,0x64,0x62}, /* 25 %      */
    {0x36,0x49,0x56,0x20,0x50}, /* 26 &      */
    {0x00,0x08,0x07,0x03,0x00}, /* 27 '      */
    {0x00,0x1C,0x22,0x41,0x00}, /* 28 (      */
    {0x00,0x41,0x22,0x1C,0x00}, /* 29 )      */
    {0x2A,0x1C,0x7F,0x1C,0x2A}, /* 2A *      */
    {0x08,0x08,0x3E,0x08,0x08}, /* 2B +      */
    {0x00,0x80,0x70,0x30,0x00}, /* 2C ,      */
    {0x08,0x08,0x08,0x08,0x08}, /* 2D -      */
    {0x00,0x00,0x60,0x60,0x00}, /* 2E .      */
    {0x20,0x10,0x08,0x04,0x02}, /* 2F /      */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 30 0      */
    {0x00,0x42,0x7F,0x40,0x00}, /* 31 1      */
    {0x72,0x49,0x49,0x49,0x46}, /* 32 2      */
    {0x21,0x41,0x49,0x4D,0x33}, /* 33 3      */
    {0x18,0x14,0x12,0x7F,0x10}, /* 34 4      */
    {0x27,0x45,0x45,0x45,0x39}, /* 35 5      */
    {0x3C,0x4A,0x49,0x49,0x31}, /* 36 6      */
    {0x41,0x21,0x11,0x09,0x07}, /* 37 7      */
    {0x36,0x49,0x49,0x49,0x36}, /* 38 8      */
    {0x46,0x49,0x49,0x29,0x1E}, /* 39 9      */
    {0x00,0x00,0x14,0x00,0x00}, /* 3A :      */
    {0x00,0x40,0x34,0x00,0x00}, /* 3B ;      */
    {0x00,0x08,0x14,0x22,0x41}, /* 3C <      */
    {0x14,0x14,0x14,0x14,0x14}, /* 3D =      */
    {0x00,0x41,0x22,0x14,0x08}, /* 3E >      */
    {0x02,0x01,0x59,0x09,0x06}, /* 3F ?      */
    {0x3E,0x41,0x5D,0x59,0x4E}, /* 40 @      */
    {0x7C,0x12,0x11,0x12,0x7C}, /* 41 A      */
    {0x7F,0x49,0x49,0x49,0x36}, /* 42 B      */
    {0x3E,0x41,0x41,0x41,0x22}, /* 43 C      */
    {0x7F,0x41,0x41,0x41,0x3E}, /* 44 D      */
    {0x7F,0x49,0x49,0x49,0x41}, /* 45 E      */
    {0x7F,0x09,0x09,0x09,0x01}, /* 46 F      */
    {0x3E,0x41,0x41,0x49,0x7A}, /* 47 G      */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 48 H      */
    {0x00,0x41,0x7F,0x41,0x00}, /* 49 I      */
    {0x20,0x40,0x41,0x3F,0x01}, /* 4A J      */
    {0x7F,0x08,0x14,0x22,0x41}, /* 4B K      */
    {0x7F,0x40,0x40,0x40,0x40}, /* 4C L      */
    {0x7F,0x02,0x1C,0x02,0x7F}, /* 4D M      */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 4E N      */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 4F O      */
    {0x7F,0x09,0x09,0x09,0x06}, /* 50 P      */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 51 Q      */
    {0x7F,0x09,0x19,0x29,0x46}, /* 52 R      */
    {0x46,0x49,0x49,0x49,0x31}, /* 53 S      */
    {0x01,0x01,0x7F,0x01,0x01}, /* 54 T      */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 55 U      */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 56 V      */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 57 W      */
    {0x63,0x14,0x08,0x14,0x63}, /* 58 X      */
    {0x07,0x08,0x70,0x08,0x07}, /* 59 Y      */
    {0x61,0x51,0x49,0x45,0x43}, /* 5A Z      */
    {0x00,0x7F,0x41,0x41,0x00}, /* 5B [      */
    {0x02,0x04,0x08,0x10,0x20}, /* 5C \      */
    {0x00,0x41,0x41,0x7F,0x00}, /* 5D ]      */
    {0x04,0x02,0x01,0x02,0x04}, /* 5E ^      */
    {0x40,0x40,0x40,0x40,0x40}, /* 5F _      */
    {0x00,0x03,0x07,0x08,0x00}, /* 60 `      */
    {0x20,0x54,0x54,0x54,0x78}, /* 61 a      */
    {0x7F,0x28,0x44,0x44,0x38}, /* 62 b      */
    {0x38,0x44,0x44,0x44,0x28}, /* 63 c      */
    {0x38,0x44,0x44,0x28,0x7F}, /* 64 d      */
    {0x38,0x54,0x54,0x54,0x18}, /* 65 e      */
    {0x00,0x08,0x7E,0x09,0x02}, /* 66 f      */
    {0x18,0xA4,0xA4,0xA4,0x7C}, /* 67 g      */
    {0x7F,0x08,0x04,0x04,0x78}, /* 68 h      */
    {0x00,0x44,0x7D,0x40,0x00}, /* 69 i      */
    {0x20,0x40,0x40,0x3D,0x00}, /* 6A j      */
    {0x7F,0x10,0x28,0x44,0x00}, /* 6B k      */
    {0x00,0x41,0x7F,0x40,0x00}, /* 6C l      */
    {0x7C,0x04,0x78,0x04,0x78}, /* 6D m      */
    {0x7C,0x08,0x04,0x04,0x78}, /* 6E n      */
    {0x38,0x44,0x44,0x44,0x38}, /* 6F o      */
    {0xFC,0x18,0x24,0x24,0x18}, /* 70 p      */
    {0x18,0x24,0x24,0x18,0xFC}, /* 71 q      */
    {0x7C,0x08,0x04,0x04,0x08}, /* 72 r      */
    {0x48,0x54,0x54,0x54,0x24}, /* 73 s      */
    {0x04,0x04,0x3F,0x44,0x24}, /* 74 t      */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 75 u      */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 76 v      */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 77 w      */
    {0x44,0x28,0x10,0x28,0x44}, /* 78 x      */
    {0x4C,0x90,0x90,0x90,0x7C}, /* 79 y      */
    {0x44,0x64,0x54,0x4C,0x44}, /* 7A z      */
    {0x00,0x08,0x36,0x41,0x00}, /* 7B {      */
    {0x00,0x00,0x77,0x00,0x00}, /* 7C |      */
    {0x00,0x41,0x36,0x08,0x00}, /* 7D }      */
    {0x02,0x01,0x02,0x04,0x02}, /* 7E ~      */
};

/* =========================================================================
 * PRIVATE HELPERS
 * ======================================================================= */

/**
 * @brief  Send a command byte to the SSD1306.
 *
 * @param  cmd  Command byte.
 * @return HAL status.
 */
static HAL_StatusTypeDef send_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00U, cmd };   /* control byte 0x00 = command mode */
    return HAL_I2C_Master_Transmit(s_hi2c, OLED_ADDR_W, buf, 2U, I2C_TIMEOUT_MS);
}

/**
 * @brief  Flush one SSD1306 page (128 bytes) from the framebuffer.
 *
 * @details Sets page address, column address = 0, then sends all
 *          128 column bytes in one I2C burst.
 *          TX buffer: [0x40 (data ctrl)] [128 pixel bytes].
 *          Total: 129 bytes per call.
 *
 * @param  page  Page index 0–7.
 * @return HAL status.
 */
static HAL_StatusTypeDef flush_page(uint8_t page)
{
    HAL_StatusTypeDef st;

    /* Set page address */
    st = send_cmd((uint8_t)(OLED_CMD_SET_PAGE_ADDR | (page & 0x07U)));
    if (st != HAL_OK) { return st; }

    /* Set column start address = 0 */
    st = send_cmd(OLED_CMD_SET_COL_LOW);
    if (st != HAL_OK) { return st; }
    st = send_cmd(OLED_CMD_SET_COL_HIGH);
    if (st != HAL_OK) { return st; }

    /* Transmit 128 data bytes prefixed by the data control byte 0x40 */
    static uint8_t tx_buf[OLED_WIDTH_PX + 1U];
    tx_buf[0] = 0x40U;   /* control byte: D/C = 1 (data), Co = 0 (stream) */
    (void)memcpy(&tx_buf[1], s_fb[page], OLED_WIDTH_PX);

    return HAL_I2C_Master_Transmit(s_hi2c, OLED_ADDR_W,
                                   tx_buf, sizeof(tx_buf), I2C_TIMEOUT_MS);
}

/**
 * @brief  Centre a string in a 21-character field, pad with spaces.
 *
 * @details Used by oled_show_message() to centre short strings.
 *          Writes into buf which must be at least 22 bytes.
 *
 * @param[out] buf   Output buffer (22 bytes minimum).
 * @param[in]  str   Input string.
 */
static void centre_str(char *buf, const char *str)
{
    uint8_t len = (uint8_t)strnlen(str, 21U);
    uint8_t pad = (21U > len) ? ((21U - len) / 2U) : 0U;

    (void)memset(buf, ' ', 21U);
    (void)memcpy(&buf[pad], str, len);
    buf[21] = '\0';
}

/* =========================================================================
 * PUBLIC API — LIFECYCLE
 * ======================================================================= */

/**
 * @brief  Initialise the SSD1306 and clear the display.
 */
MmResult_t oled_init(void *hi2c)
{
    if (hi2c == NULL) { return MM_ERR_PARAM; }
    s_hi2c = (I2C_HandleTypeDef *)hi2c;

    HAL_Delay(50U);   /* SSD1306 power-on stabilisation */

    /* SSD1306 initialisation sequence — 25 commands */
    static const uint8_t INIT_CMDS[] = {
        OLED_CMD_DISPLAY_OFF,
        OLED_CMD_SET_DISP_CLK,    0x80U,   /* clock div = 1, osc freq = 8 */
        OLED_CMD_SET_MULTIPLEX,   0x3FU,   /* 64 MUX                       */
        OLED_CMD_SET_DISP_OFFSET, 0x00U,   /* no offset                    */
        OLED_CMD_SET_START_LINE,           /* start line = 0               */
        OLED_CMD_CHARGE_PUMP,     0x14U,   /* charge pump ON               */
        OLED_CMD_SET_MEM_ADDR_MODE, OLED_CMD_ADDR_MODE_HORIZ,
        OLED_CMD_SEG_REMAP,                /* segment remap                */
        OLED_CMD_COM_SCAN_DEC,             /* COM scan direction           */
        OLED_CMD_SET_COM_PINS,    0x12U,   /* alt COM config, no remap     */
        OLED_CMD_SET_CONTRAST,    0xCFU,   /* full contrast                */
        OLED_CMD_SET_PRECHARGE,   0xF1U,   /* pre-charge period            */
        OLED_CMD_SET_VCOM,        0x30U,   /* VCOMH = 0.83 × Vcc          */
        OLED_CMD_ENTIRE_DISP_OFF,          /* output follows RAM           */
        OLED_CMD_NORMAL_DISP,              /* not inverted                 */
        OLED_CMD_DISPLAY_ON,               /* turn display on              */
    };

    for (uint8_t i = 0U; i < (uint8_t)sizeof(INIT_CMDS); i++)
    {
        if (send_cmd(INIT_CMDS[i]) != HAL_OK)
        {
            return MM_ERR_DRIVER;
        }
    }

    oled_clear();
    return oled_flush();
}

void oled_clear(void)
{
    (void)memset(s_fb, 0x00U, sizeof(s_fb));
}

void oled_clear_flush(void)
{
    oled_clear();
    (void)oled_flush();
}

MmResult_t oled_flush(void)
{
    for (uint8_t p = 0U; p < OLED_PAGES; p++)
    {
        if (flush_page(p) != HAL_OK)
        {
            return MM_ERR_DRIVER;
        }
    }
    return MM_OK;
}

/* =========================================================================
 * PUBLIC API — PRIMITIVE DRAWING
 * ======================================================================= */

/**
 * @brief  Set or clear a single pixel.
 */
void oled_pixel(uint8_t x, uint8_t y, bool on)
{
    if (x >= OLED_WIDTH_PX || y >= OLED_HEIGHT_PX) { return; }

    uint8_t page = y / 8U;
    uint8_t bit  = y % 8U;

    if (on)
        s_fb[page][x] |=  (uint8_t)(1U << bit);
    else
        s_fb[page][x] &= ~(uint8_t)(1U << bit);
}

/**
 * @brief  Draw a horizontal line.
 */
void oled_hline(uint8_t x, uint8_t y, uint8_t len, bool on)
{
    uint8_t end = (uint8_t)((x + len < OLED_WIDTH_PX) ? x + len : OLED_WIDTH_PX);
    for (uint8_t col = x; col < end; col++)
    {
        oled_pixel(col, y, on);
    }
}

/**
 * @brief  Draw a filled rectangle.
 */
void oled_rect_fill(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{
    uint8_t y_end = (uint8_t)((y + h < OLED_HEIGHT_PX) ? y + h : OLED_HEIGHT_PX);
    uint8_t x_end = (uint8_t)((x + w < OLED_WIDTH_PX)  ? x + w : OLED_WIDTH_PX);

    for (uint8_t row = y; row < y_end; row++)
    {
        for (uint8_t col = x; col < x_end; col++)
        {
            oled_pixel(col, row, on);
        }
    }
}

/* =========================================================================
 * PUBLIC API — TEXT RENDERING
 * ======================================================================= */

/**
 * @brief  Draw a single character into the framebuffer.
 */
void oled_char(uint8_t x, uint8_t page, char ch)
{
    if (page >= OLED_PAGES)          { return; }
    if (x + 6U > OLED_WIDTH_PX)     { return; }   /* won't fit           */

    uint8_t idx = (uint8_t)ch;
    if (idx < 0x20U || idx > 0x7EU) { idx = 0x20U; }   /* default: space */
    idx -= 0x20U;   /* map to font table index */

    for (uint8_t col = 0U; col < 5U; col++)
    {
        s_fb[page][x + col] = FONT6x8[idx][col];
    }
    s_fb[page][x + 5U] = 0x00U;   /* 1-pixel spacer column              */
}

/**
 * @brief  Draw a null-terminated string.
 */
void oled_str(uint8_t x, uint8_t page, const char *str)
{
    if (str == NULL) { return; }
    while (*str != '\0' && x + 6U <= OLED_WIDTH_PX)
    {
        oled_char(x, page, *str);
        x   += 6U;
        str ++;
    }
}

/**
 * @brief  Printf-style text render — max 21 characters.
 */
void oled_printf(uint8_t x, uint8_t page, const char *fmt, ...)
{
    char    buf[22];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    oled_str(x, page, buf);
}

/**
 * @brief  Clear a page and write a string from column 0.
 */
void oled_line(uint8_t page, const char *str)
{
    if (page >= OLED_PAGES) { return; }
    (void)memset(s_fb[page], 0x00U, OLED_WIDTH_PX);
    oled_str(0U, page, str);
}

/**
 * @brief  Clear a page and write a formatted string.
 */
void oled_linef(uint8_t page, const char *fmt, ...)
{
    char    buf[22];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    oled_line(page, buf);
}

/* =========================================================================
 * PUBLIC API — STATUS SCREENS
 * ======================================================================= */

void oled_show_boot(void)
{
    oled_clear();
    oled_line(0U, "MicroMaze 3");
    oled_linef(1U, "STM32F411 v%u.%u.%u",
               FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    oled_line(2U, "");
    oled_line(3U, "INITIALISING...");
    (void)oled_flush();
}

void oled_show_boot_complete(uint8_t mode, float batt_v, bool cal_loaded)
{
    static const char *const MODE_NAMES[16] = {
        "MONITOR",  "IR CAL",  "MOTOR TEST","STRAIGHT",
        "TURN TEST","WALL FOL","SEARCH RUN","SPEED 1",
        "SPEED 2",  "SPEED 3", "AUTO QUAL", "GYRO DBG",
        "PRINT MAP","BATT CHK","SELF TEST", "RESERVED",
    };
    oled_clear();
    oled_line(0U, "READY");
    oled_linef(1U, "Batt:%.2fV", batt_v);
    oled_line(2U, cal_loaded ? "IR:calibrated" : "IR:defaults!");
    oled_linef(3U, "Mode%u:%s", mode, MODE_NAMES[mode & 0x0FU]);
    oled_line(4U, "Press BTN to run");
    (void)oled_flush();
}

void oled_show_mode(uint8_t mode)
{
    static const char *const MODE_NAMES[16] = {
        "MONITOR",  "IR CAL",  "MOTOR TEST","STRAIGHT",
        "TURN TEST","WALL FOL","SEARCH RUN","SPEED 1",
        "SPEED 2",  "SPEED 3", "AUTO QUAL", "GYRO DBG",
        "PRINT MAP","BATT CHK","SELF TEST", "RESERVED",
    };
    oled_clear();
    oled_linef(0U, "MODE %u", mode);
    oled_line(1U, MODE_NAMES[mode & 0x0FU]);
    oled_line(2U, "Press BTN to run");
    (void)oled_flush();
}

void oled_show_sensors(const uint16_t diff[6],
                       bool wall_f, bool wall_l, bool wall_r,
                       float front_err, float side_err)
{
    oled_clear();
    oled_line(0U, "SENSORS");
    oled_linef(1U, "FL:%4u FR:%4u", diff[IR_RF], diff[IR_LF]);
    oled_linef(2U, "LS:%4u RS:%4u", diff[IR_LS], diff[IR_RS]);
    oled_linef(3U, "LA:%4u RA:%4u", diff[IR_L_ANG], diff[IR_R_ANG]);
    oled_linef(4U, "W:%c%c%c",
               wall_f ? 'F' : '.', wall_l ? 'L' : '.', wall_r ? 'R' : '.');
    oled_linef(5U, "FErr:%.0f", (double)front_err);
    oled_linef(6U, "SErr:%.0f", (double)side_err);
    (void)oled_flush();
}

void oled_show_motion(float    spd_l_mmps,
                      float    spd_r_mmps,
                      float    pos_mm,
                      float    yaw_deg,
                      float    track_err_mm,
                      uint8_t  maze_row,
                      uint8_t  maze_col,
                      uint8_t  heading,
                      uint32_t elapsed_ms,
                      float    batt_v)
{
    static const char HEADING_CHAR[4] = { 'N', 'E', 'S', 'W' };
    uint32_t secs = elapsed_ms / 1000UL;

    oled_clear();
    oled_linef(0U, "RUN  %lus", (unsigned long)secs);
    oled_linef(1U, "L:%.0f R:%.0f", (double)spd_l_mmps, (double)spd_r_mmps);
    oled_linef(2U, "Pos:%.1f mm",   (double)pos_mm);
    oled_linef(3U, "Yaw:%.2f deg",  (double)yaw_deg);
    oled_linef(4U, "Err:%.2f mm",   (double)track_err_mm);
    oled_linef(5U, "R:%02u C:%02u %c",
               maze_row, maze_col,
               HEADING_CHAR[heading < 4U ? heading : 0U]);
    oled_linef(6U, "Bat:%.2fV",     (double)batt_v);
    (void)oled_flush();
}

void oled_show_gyro(float yaw_deg, float rate_dps, float offset_dps)
{
    oled_clear();
    oled_line(0U, "GYRO DEBUG");
    oled_linef(1U, "Yaw:%.2f deg",  (double)yaw_deg);
    oled_linef(2U, "Rate:%.2f d/s", (double)rate_dps);
    oled_linef(3U, "Off:%.4f",      (double)offset_dps);
    (void)oled_flush();
}

void oled_show_cal(uint8_t step, const uint16_t thresh[6])
{
    static const char *const STEPS[4] = {
        "", "Step1:open space", "Step2:3-wall cell", "SAVED to Flash"
    };
    oled_clear();
    oled_line(0U, "IR CALIBRATE");
    oled_line(1U, (step < 4U) ? STEPS[step] : "");
    oled_line(2U, "Press BTN");
    if (thresh != NULL && step >= 2U)
    {
        oled_linef(3U, "FL%3u FR%3u", thresh[IR_RF], thresh[IR_LF]);
        oled_linef(4U, "LS%3u RS%3u", thresh[IR_LS], thresh[IR_RS]);
        oled_linef(5U, "LA%3u RA%3u", thresh[IR_L_ANG], thresh[IR_R_ANG]);
    }
    (void)oled_flush();
}

void oled_show_battery(float volts, uint8_t percent)
{
    static const char *status_str(float v) {
        if (v < 6.6f)  return "CRITICAL!";
        if (v < 7.0f)  return "LOW";
        if (v < 7.8f)  return "OK";
        return "FULL";
    }

    oled_clear();
    oled_line(0U, "BATTERY");
    oled_linef(1U, "Voltage: %.2fV",  (double)volts);
    oled_linef(2U, "Level:   %u%%",    percent);
    oled_line(3U, status_str(volts));

    /* Bar graph: pages 5–6, proportional to percent */
    uint8_t bar_w = (uint8_t)(((uint16_t)percent * OLED_WIDTH_PX) / 100U);
    oled_rect_fill(0U, 40U, bar_w, 16U, true);
    oled_rect_fill(bar_w, 40U, (uint8_t)(OLED_WIDTH_PX - bar_w), 16U, false);

    (void)oled_flush();
}

void oled_show_maze(uint8_t row, uint8_t col, uint8_t heading, uint8_t flood,
                    bool wall_f, bool wall_l, bool wall_r)
{
    static const char HEADING_STR[4][2] = { "N", "E", "S", "W" };
    oled_clear();
    oled_linef(0U, "MAZE R:%02u C:%02u", row, col);
    oled_linef(1U, "Head: %s",
               HEADING_STR[heading < 4U ? heading : 0U]);
    oled_linef(2U, "Flood: %3u", flood);
    oled_linef(3U, "W:%c%c%c",
               wall_f ? 'F' : '.', wall_l ? 'L' : '.', wall_r ? 'R' : '.');
    (void)oled_flush();
}

void oled_show_message(const char *line1, const char *line2)
{
    char buf[22];
    oled_clear();

    centre_str(buf, (line1 != NULL) ? line1 : "");
    oled_line(3U, buf);

    if (line2 != NULL)
    {
        centre_str(buf, line2);
        oled_line(4U, buf);
    }

    (void)oled_flush();
}

void oled_show_error(const char *message, int32_t code)
{
    oled_clear();
    oled_line(0U, "!! ERROR !!");
    oled_line(1U, (message != NULL) ? message : "Unknown");
    oled_linef(2U, "Code: %ld", (long)code);
    oled_line(3U, "Reset to recover");
    (void)oled_flush();
}

/* =========================================================================
 * PUBLIC API — LOW-LEVEL ACCESS
 * ======================================================================= */

MmResult_t oled_send_cmd(uint8_t cmd)
{
    return (send_cmd(cmd) == HAL_OK) ? MM_OK : MM_ERR_DRIVER;
}

const uint8_t *oled_get_framebuffer(void)
{
    return &s_fb[0][0];
}

#endif /* OLED_ENABLED */
