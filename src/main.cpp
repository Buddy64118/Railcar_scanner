// ============================================================
// Railroad Car Identification Scanner
// Hardware: Raspberry Pi Pico W
//           Waveshare Barcode Scanner Module (UART1)
//           0.96" OLED Display (I2C0, SSD1306)
//           Momentary Push Button (Scan Trigger) — GP15
//
// Connector Groups (Keyestudio Pico Breakout Board):
//   UART1 group : GP4 (TX) | GP5 (RX)              -> Scanner
//   I2C0 group  : GP20 (SDA) | GP21 (SCL)          -> OLED
//
// QR Code Data Format (pipe-delimited, no spaces):
//   REPORTING_MARK|CAR_NUMBER|CAR_TYPE|LOADED_WEIGHT
//   Example: BNSF|123456|Boxcar|47.3
// ============================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

// ---- Pin Definitions ----------------------------------------

// UART1 (Barcode Scanner) — GP4/GP5 are wired to UART1 on the RP2040
#define SCANNER_UART        uart1
#define SCANNER_UART_TX     4       // GP4 → Scanner RX
#define SCANNER_UART_RX     5       // GP5 ← Scanner TX
#define SCANNER_BAUD        9600

// I2C0 (OLED Display) — GP20/GP21 are wired to I2C0 on the RP2040
#define OLED_I2C            i2c0
#define OLED_SDA_PIN        20      // GP20 — I2C0 SDA
#define OLED_SCL_PIN        21      // GP21 — I2C0 SCL
#define OLED_I2C_ADDR       0x3C
#define OLED_I2C_FREQ       400000

// I2C timeout values (microseconds)
#define I2C_CMD_TIMEOUT_US  10000   // 10ms for single command bytes
#define I2C_DATA_TIMEOUT_US 25000   // 25ms for full page data writes

// GPIO Controls
#define SCAN_BUTTON_PIN     15      // GP15 — Momentary button: active LOW (GND)

// ============================================================
// Tunable Parameters — adjust these to fine-tune behavior
// ============================================================
#define SCAN_TIMEOUT_MS         10000   // How long to wait for scanner response
#define SCAN_DEBOUNCE_MS        50      // Button debounce delay
#define SCAN_ERROR_HOLD_MS      2000    // How long error screen stays visible
#define OLED_CONTRAST           0xCF    // Display brightness 0x00-0xFF

// ---- SSD1306 OLED Constants ---------------------------------
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGES          8       // 64px / 8px per page

// SSD1306 Commands
#define OLED_CMD_DISPLAY_OFF        0xAE
#define OLED_CMD_DISPLAY_ON         0xAF
#define OLED_CMD_SET_CONTRAST       0x81
// 0xA4 = output follows RAM contents (correct for normal operation).
// 0xA5 would force ALL pixels on regardless of frame buffer.
#define OLED_CMD_ENTIRE_DISPLAY_ON  0xA4
#define OLED_CMD_NORMAL_DISPLAY     0xA6
#define OLED_CMD_SET_MUX_RATIO      0xA8
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3
#define OLED_CMD_SET_DISPLAY_CLK    0xD5
#define OLED_CMD_SET_PRECHARGE      0xD9
#define OLED_CMD_SET_COM_PINS       0xDA
#define OLED_CMD_SET_VCOM_DESELECT  0xDB
#define OLED_CMD_SET_CHARGE_PUMP    0x8D
#define OLED_CMD_MEM_ADDR_MODE      0x20
#define OLED_CMD_SET_COL_ADDR       0x21
#define OLED_CMD_SET_PAGE_ADDR      0x22
#define OLED_CMD_SET_START_LINE     0x40
#define OLED_CMD_SEG_REMAP          0xA1
#define OLED_CMD_COM_OUT_DIR        0xC8

// ---- Frame Buffer -------------------------------------------
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];

// ---- 5x7 Font (ASCII 32-127) --------------------------------
// Each character is 5 bytes wide, 7 pixels tall (1 byte per column)
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' ' (32)
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x10,0x08,0x08,0x10,0x08}, // '~'
    {0x00,0x00,0x00,0x00,0x00}, // DEL
};

// ====================================================================
// OLED Low-Level Functions
// All I2C writes use timeout versions to prevent the main loop
// from hanging if the I2C bus enters a bad state.
// ====================================================================

static void oled_write_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};  // 0x00 = control byte for command
    i2c_write_timeout_us(OLED_I2C, OLED_I2C_ADDR,
                         buf, 2, false, I2C_CMD_TIMEOUT_US);
}

// Batch all address-setup commands into a single I2C transaction
// instead of separate start/stop pairs for efficiency.
static void oled_flush(void) {
    uint8_t setup[7];
    setup[0] = 0x00;                    // control byte — commands follow
    setup[1] = OLED_CMD_SET_COL_ADDR;
    setup[2] = 0;
    setup[3] = OLED_WIDTH - 1;
    setup[4] = OLED_CMD_SET_PAGE_ADDR;
    setup[5] = 0;
    setup[6] = OLED_PAGES - 1;
    i2c_write_timeout_us(OLED_I2C, OLED_I2C_ADDR,
                         setup, sizeof(setup), false, I2C_CMD_TIMEOUT_US);

    for (int page = 0; page < OLED_PAGES; page++) {
        uint8_t buf[OLED_WIDTH + 1];
        buf[0] = 0x40;  // Control byte: data follows
        memcpy(&buf[1], &oled_buffer[page * OLED_WIDTH], OLED_WIDTH);
        i2c_write_timeout_us(OLED_I2C, OLED_I2C_ADDR,
                             buf, OLED_WIDTH + 1, false, I2C_DATA_TIMEOUT_US);
    }
}

static void oled_init(void) {
    sleep_ms(100);  // Allow OLED power rail to stabilize
    oled_write_cmd(OLED_CMD_DISPLAY_OFF);
    oled_write_cmd(OLED_CMD_SET_DISPLAY_CLK);    oled_write_cmd(0x80);
    oled_write_cmd(OLED_CMD_SET_MUX_RATIO);      oled_write_cmd(0x3F);
    oled_write_cmd(OLED_CMD_SET_DISPLAY_OFFSET); oled_write_cmd(0x00);
    oled_write_cmd(OLED_CMD_SET_START_LINE | 0x00);
    oled_write_cmd(OLED_CMD_SET_CHARGE_PUMP);    oled_write_cmd(0x14);
    oled_write_cmd(OLED_CMD_MEM_ADDR_MODE);      oled_write_cmd(0x00);
    oled_write_cmd(OLED_CMD_SEG_REMAP);
    oled_write_cmd(OLED_CMD_COM_OUT_DIR);
    oled_write_cmd(OLED_CMD_SET_COM_PINS);       oled_write_cmd(0x12);
    oled_write_cmd(OLED_CMD_SET_CONTRAST);       oled_write_cmd(OLED_CONTRAST);
    oled_write_cmd(OLED_CMD_SET_PRECHARGE);      oled_write_cmd(0xF1);
    oled_write_cmd(OLED_CMD_SET_VCOM_DESELECT);  oled_write_cmd(0x40);
    oled_write_cmd(OLED_CMD_ENTIRE_DISPLAY_ON);  // 0xA4 = follow RAM
    oled_write_cmd(OLED_CMD_NORMAL_DISPLAY);
    oled_write_cmd(OLED_CMD_DISPLAY_ON);
}

static void oled_clear(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

// Draw a single character at pixel column x, page row (0-7)
static void oled_draw_char(uint8_t x, uint8_t page, char c) {
    if (c < 32 || c > 127) c = ' ';
    const uint8_t *glyph = font5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        if (x + col < OLED_WIDTH) {
            oled_buffer[page * OLED_WIDTH + x + col] = glyph[col];
        }
    }
    if (x + 5 < OLED_WIDTH) {
        oled_buffer[page * OLED_WIDTH + x + 5] = 0x00;
    }
}

// Draw a string; each character is 6 pixels wide (5 glyph + 1 gap)
static void oled_draw_string(uint8_t x, uint8_t page, const char *str) {
    while (*str && x < OLED_WIDTH) {
        oled_draw_char(x, page, *str++);
        x += 6;
    }
}

// Draw a solid horizontal separator line across the full display width
static void oled_draw_separator(uint8_t page) {
    for (int x = 0; x < OLED_WIDTH; x++) {
        oled_buffer[page * OLED_WIDTH + x] = 0xFF;
    }
}

// Draw a single character at 2x scale (10x14 pixels per character)
static void oled_draw_char_large(uint8_t x, uint8_t page, char c) {
    if (c < 32 || c > 127) c = ' ';
    const uint8_t *glyph = font5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        uint8_t col_data = glyph[col];
        // Expand each bit vertically into two rows (two pages)
        uint8_t top = 0, bot = 0;
        for (int bit = 0; bit < 4; bit++) {
            if (col_data & (1 << bit))       top |= (3 << (bit * 2));
            if (col_data & (1 << (bit + 4))) bot |= (3 << ((bit - 4) * 2 + 8));
        }
        // Write two pixels wide per column
        for (int dx = 0; dx < 2; dx++) {
            if (x + col*2 + dx < OLED_WIDTH) {
                oled_buffer[page       * OLED_WIDTH + x + col*2 + dx] = top;
                oled_buffer[(page + 1) * OLED_WIDTH + x + col*2 + dx] = bot;
            }
        }
    }
    // 2-pixel gap between characters
    for (int dx = 0; dx < 2; dx++) {
        if (x + 10 + dx < OLED_WIDTH) {
            oled_buffer[page       * OLED_WIDTH + x + 10 + dx] = 0x00;
            oled_buffer[(page + 1) * OLED_WIDTH + x + 10 + dx] = 0x00;
        }
    }
}

// Draw a string at 2x scale; each character is 12 pixels wide
static void oled_draw_string_large(uint8_t x, uint8_t page, const char *str) {
    while (*str && x < OLED_WIDTH) {
        oled_draw_char_large(x, page, *str++);
        x += 12;
    }
}

// ====================================================================
// Car Data Structure
// ====================================================================
typedef struct {
    char  reporting_mark[16];
    char  car_number[12];
    char  car_type[24];
    char  loaded_weight[12]; // stored as string for display
    float loaded_weight_f;   // parsed float for validation
} CarData;

// ====================================================================
// String Utility — strip trailing whitespace and control characters
// ====================================================================
static void trim(char *s) {
    int len = (int)strlen(s);
    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' ||
                       s[len-1] == ' '  || s[len-1] == '\t')) {
        s[--len] = '\0';
    }
}

// ====================================================================
// QR Code Parser
// Splits a pipe-delimited string into the four CarData fields.
// Strips any leading non-alphabetic characters from reporting mark
// to handle scanner AIM ID prefix bytes.
// Returns true if all four fields were successfully found and valid.
// ====================================================================
static bool parse_car_data(const char *raw, CarData *car) {
    memset(car, 0, sizeof(CarData));
    char buf[128];
    strncpy(buf, raw, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tok = strtok(buf, "|");
    if (!tok) return false;
    strncpy(car->reporting_mark, tok, sizeof(car->reporting_mark) - 1);
    trim(car->reporting_mark);

    // Strip any leading non-alphabetic characters (e.g. AIM ID prefix)
    char *mark_start = car->reporting_mark;
    while (*mark_start && !isalpha((unsigned char)*mark_start)) {
        mark_start++;
    }
    if (mark_start != car->reporting_mark) {
        memmove(car->reporting_mark, mark_start, strlen(mark_start) + 1);
    }

    tok = strtok(NULL, "|");
    if (!tok) return false;
    strncpy(car->car_number, tok, sizeof(car->car_number) - 1);
    trim(car->car_number);

    tok = strtok(NULL, "|");
    if (!tok) return false;
    strncpy(car->car_type, tok, sizeof(car->car_type) - 1);
    trim(car->car_type);

    tok = strtok(NULL, "|");
    if (!tok) return false;
    strncpy(car->loaded_weight, tok, sizeof(car->loaded_weight) - 1);
    trim(car->loaded_weight);

    // Validate weight field is a real number
    char *endptr;
    car->loaded_weight_f = strtof(car->loaded_weight, &endptr);
    if (endptr == car->loaded_weight || car->loaded_weight_f < 0.0f) {
        printf("Weight field invalid: [%s]\n", car->loaded_weight);
        return false;
    }

    // Normalise display string to 1 decimal place
    snprintf(car->loaded_weight, sizeof(car->loaded_weight),
             "%.1f", car->loaded_weight_f);

    return true;
}

// ====================================================================
// Display Screens
// ====================================================================

static void display_splash(void) {
    oled_clear();
    // Pages 0-1: Title in yellow zone
    oled_draw_string_large(4, 0, "RR SCALE");
    // Pages 2-7: Instructions in blue zone
    oled_draw_separator(2);
    oled_draw_string(8, 4, "Place car on scale");
    oled_draw_string(22, 6, "Press SCAN");
    oled_flush();
}

static void display_car_data(const CarData *car) {
    oled_clear();

    // Pages 0-1: Reporting mark (YELLOW zone)
    oled_draw_string_large(0, 0, car->reporting_mark);

    // Pages 2-3: Car number (BLUE zone)
    oled_draw_string_large(0, 2, car->car_number);

    // Pages 4-5: Car type
    oled_draw_string_large(0, 4, car->car_type);

    // Pages 6-7: Weight with OK indicator at far right
    char wt_line[16];
    snprintf(wt_line, sizeof(wt_line), "%s T", car->loaded_weight);
    oled_draw_string_large(0, 6, wt_line);
    oled_draw_string(110, 7, "OK");

    oled_flush();
}

static void display_error(const char *msg) {
    oled_clear();
    oled_draw_string(0, 0, "  SCAN ERROR");
    oled_draw_separator(1);
    oled_draw_string(0, 3, msg);
    oled_draw_separator(5);
    oled_draw_string(0, 7, "Try again...");
    oled_flush();
}

// ====================================================================
// Scanner UART Functions
// ====================================================================

// Send the software trigger command and discard the acknowledgment
// response bytes before returning so scanner_readline() sees only
// the actual barcode data.
static void scanner_send_trigger(void) {
    const uint8_t cmd[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
    uart_write_blocking(SCANNER_UART, cmd, sizeof(cmd));
    printf("Trigger command sent\n");

    // Discard 7-byte acknowledgment: 02 00 00 01 00 33 31
uint8_t ack[7];
    int ack_received = 0;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (ack_received < 7) {
        if (uart_is_readable(SCANNER_UART)) {
            ack[ack_received++] = uart_getc(SCANNER_UART);
        }
        if (to_ms_since_boot(get_absolute_time()) - start > 2000) break;
        sleep_us(100);
    }
    printf("Acknowledgment received (%d bytes)\n", ack_received);
}

// Read a line of UART data into buf (up to max_len-1 characters).
// Stops at newline, carriage return, or when timeout_ms expires.
// Returns the number of bytes read (0 = nothing received).
static int scanner_readline(char *buf, int max_len, uint32_t timeout_ms) {
    int idx = 0;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (idx < max_len - 1) {
        if (uart_is_readable(SCANNER_UART)) {
            char c = uart_getc(SCANNER_UART);
            if (c == '\r' || c == '\n') {
                if (idx > 0) break;
            } else {
                buf[idx++] = c;
            }
        } else {
            sleep_us(100);
        }
        if (to_ms_since_boot(get_absolute_time()) - start > timeout_ms) break;
    }
    buf[idx] = '\0';
    if (idx > 0) {
        printf("Raw UART data received: [%s] (%d bytes)\n", buf, idx);
    }
    return idx;
}

// ====================================================================
// Main
// ====================================================================

int main(void) {
    stdio_init_all();
    sleep_ms(3000);  // Wait for USB serial to enumerate
    printf("Railroad Car Scanner starting...\n");

    // ---- I2C init for OLED ----
    i2c_init(OLED_I2C, OLED_I2C_FREQ);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);
    printf("I2C initialized on GP%d (SDA) and GP%d (SCL)\n",
           OLED_SDA_PIN, OLED_SCL_PIN);

    // ---- UART init for barcode scanner ----
    uart_init(SCANNER_UART, SCANNER_BAUD);
    gpio_set_function(SCANNER_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(SCANNER_UART_RX, GPIO_FUNC_UART);
    printf("UART1 initialized on GP%d (TX) and GP%d (RX) at %d baud\n",
           SCANNER_UART_TX, SCANNER_UART_RX, SCANNER_BAUD);

    // ---- GPIO: Scan button (GP15, active LOW) ----
    gpio_init(SCAN_BUTTON_PIN);
    gpio_set_dir(SCAN_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(SCAN_BUTTON_PIN);

    // ---- Start OLED ----
    oled_init();
    printf("OLED initialized\n");

    // ---- Flush UART buffer before entering main loop ----
    sleep_ms(200);
    while (uart_is_readable(SCANNER_UART)) uart_getc(SCANNER_UART);

    display_splash();
    printf("Entering main loop\n");

    char scan_buf[128];
    CarData car;

    while (true) {

        if (!gpio_get(SCAN_BUTTON_PIN)) {
            sleep_ms(SCAN_DEBOUNCE_MS);
            if (!gpio_get(SCAN_BUTTON_PIN)) {
                printf("Scan button pressed\n");
                scanner_send_trigger();

                // Wait for button release before reading data
                while (!gpio_get(SCAN_BUTTON_PIN)) sleep_ms(10);

                // Listen for scanner response
                int len = scanner_readline(scan_buf, sizeof(scan_buf), SCAN_TIMEOUT_MS);
                if (len > 0) {
                    if (parse_car_data(scan_buf, &car)) {
                        printf("Car: %s | %s | %s | %.1f T\n",
                               car.reporting_mark, car.car_number,
                               car.car_type, car.loaded_weight_f);
                        display_car_data(&car);
                    } else {
                        printf("Parse failed for: [%s]\n", scan_buf);
                        display_error("Bad QR format");
                        sleep_ms(SCAN_ERROR_HOLD_MS);
                        display_splash();
                    }
                } else {
                    printf("No data received within timeout\n");
                    display_error("No scan data");
                    sleep_ms(SCAN_ERROR_HOLD_MS);
                    display_splash();
                }
            }
        }

        sleep_ms(10);

    } // end while(true)

    return 0;
}
