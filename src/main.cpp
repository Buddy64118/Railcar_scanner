// ============================================================
// Railroad Car Identification Scanner
// Hardware: Raspberry Pi Pico W
//           Waveshare Barcode Scanner Module (UART0)
//           0.96" OLED Display (I2C0, SSD1306)
//           Toggle Switch (Command/Continuous Mode) — GP14
//           Momentary Push Button (Scan Trigger) — GP15
//
// Connector Groups (Keyestudio Pico Breakout Board):
//   UART0 group : 3v3 | GND | GP1 (RX) | GP0 (TX)  -> Scanner
//   I2C0 group  : GP21 (SCL) | GP20 (SDA) | 3v3 | GND -> OLED
//
// QR Code Data Format (pipe-delimited, no spaces):
//   REPORTING_MARK|CAR_NUMBER|CAR_TYPE|LOADED_WEIGHT
//   Example: BNSF|123456|Boxcar|47.3
// ============================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

// ---- Pin Definitions ----------------------------------------

// UART1 (Barcode Scanner) — GP4/GP5 are wired to UART1 on the RP2040
#define SCANNER_UART        uart1
#define SCANNER_UART_TX     4       // GP4 → Scanner RX
#define SCANNER_UART_RX     5       // GP5 ← Scanner TX
#define SCANNER_BAUD        9600

// I2C0 (OLED) — GP20/GP21 are wired to I2C0 on the RP2040
#define OLED_I2C            i2c0
#define OLED_SDA_PIN        20     // GP20 → OLED SDA          
#define OLED_SCL_PIN        21     // GP21 → OLED SCL                          
#define OLED_I2C_ADDR       0x3C
#define OLED_I2C_FREQ       400000

// I2C timeout values (microseconds)
#define I2C_CMD_TIMEOUT_US  10000   // 10ms for single command bytes
#define I2C_DATA_TIMEOUT_US 25000   // 25ms for full page data writes

// GPIO Controls
#define MODE_TOGGLE_PIN     14      // GP14 — Toggle: open=Command, GND=Continuous
#define SCAN_BUTTON_PIN     15      // GP15 — Momentary button: active LOW (GND)

// ---- SSD1306 OLED Constants ---------------------------------
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGES          8       // 64px / 8px per page

// SSD1306 Commands
#define OLED_CMD_DISPLAY_OFF        0xAE
#define OLED_CMD_DISPLAY_ON         0xAF
#define OLED_CMD_SET_CONTRAST       0x81
// FIX (Bug 2): 0xA4 = output follows RAM contents (correct for normal operation).
// 0xA5 would force ALL pixels on regardless of frame buffer — never use 0xA5 here.
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

// FIX (Tip 1): Batch all 6 address-setup commands into a single I2C
// transaction instead of 6 separate start/stop pairs.  At 400 kHz this
// saves ~450 µs per flush — a worthwhile reduction at 60 fps+ rates.
static void oled_flush(void) {
    // Build one command buffer: control byte + 6 setup bytes
    uint8_t setup[7];
    setup[0] = 0x00;                        // control byte — commands follow
    setup[1] = OLED_CMD_SET_COL_ADDR;
    setup[2] = 0;
    setup[3] = OLED_WIDTH - 1;
    setup[4] = OLED_CMD_SET_PAGE_ADDR;
    setup[5] = 0;
    setup[6] = OLED_PAGES - 1;
    i2c_write_timeout_us(OLED_I2C, OLED_I2C_ADDR,
                         setup, sizeof(setup), false, I2C_CMD_TIMEOUT_US);

    // Send each page of the frame buffer using a timeout write.
    // If the bus locks up, the timeout prevents an infinite hang.
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
    oled_write_cmd(OLED_CMD_SET_CONTRAST);       oled_write_cmd(0xCF);
    oled_write_cmd(OLED_CMD_SET_PRECHARGE);      oled_write_cmd(0xF1);
    oled_write_cmd(OLED_CMD_SET_VCOM_DESELECT);  oled_write_cmd(0x40);
    oled_write_cmd(OLED_CMD_ENTIRE_DISPLAY_ON);  // 0xA4 = follow RAM (see #define)
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
    // 1-pixel gap between characters
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

// ====================================================================
// Car Data Structure
// ====================================================================
typedef struct {
    char  reporting_mark[16];
    char  car_number[12];
    char  car_type[24];
    char  loaded_weight[12]; // stored as string for display
    float loaded_weight_f;   // FIX (Tip 3): parsed float for validation
} CarData;

// ====================================================================
// String Utility — strip trailing whitespace and control characters
// Handles \r, \n, space, and tab left behind by QR code generators
// or the scanner module's line terminator.
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
// FIX (Tip 2): Removed redundant whole-string trim before strtok —
//   only per-token trims are needed since strtok splits on '|', not
//   whitespace, so trimming the whole string only affected the last
//   field anyway.
// FIX (Tip 3): Parses loaded_weight into a float and validates it.
//   Returns false if the weight field is not a valid positive number.
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

    // FIX (Tip 3): Validate weight field is a real number.
    // strtof sets endptr to tok if no conversion was possible.
    char *endptr;
    car->loaded_weight_f = strtof(car->loaded_weight, &endptr);
    if (endptr == car->loaded_weight || car->loaded_weight_f < 0.0f) {
        printf("Weight field invalid: [%s]\n", car->loaded_weight);
        return false;
    }

    // Normalise display string to 1 decimal place regardless of QR formatting
    snprintf(car->loaded_weight, sizeof(car->loaded_weight),
             "%.1f", car->loaded_weight_f);

    return true;
}

// ====================================================================
// Display Screens
// ====================================================================

static void display_splash(bool continuous_mode) {
    oled_clear();
    oled_draw_string(4,  0, "RR CAR SCANNER");
    oled_draw_separator(1);
    oled_draw_string(4,  2, "Scan a QR Code");
    oled_draw_string(4,  3, " to identify");
    oled_draw_string(4,  4, "  a railcar.");
    oled_draw_separator(5);
    if (continuous_mode) {
        oled_draw_string(0, 6, "Mode: CONT");
        oled_draw_string(0, 7, "Auto-scanning...");
    } else {
        oled_draw_string(0, 6, "Mode: CMD");
        oled_draw_string(0, 7, "Press SCAN btn");
    }
    oled_flush();
}

static void display_car_data(const CarData *car, bool continuous_mode) {
    oled_clear();

    // Page 0: Reporting mark + car number
    char header[22];
    snprintf(header, sizeof(header), "%s #%s", car->reporting_mark, car->car_number);
    oled_draw_string(0, 0, header);
    oled_draw_separator(1);

    // Page 2: Car type
    char type_line[22];
    snprintf(type_line, sizeof(type_line), "Type: %s", car->car_type);
    oled_draw_string(0, 2, type_line);

    // Page 3: Loaded weight
    char wt_line[22];
    snprintf(wt_line, sizeof(wt_line), "Wt: %s T", car->loaded_weight);
    oled_draw_string(0, 3, wt_line);

    // Separator on page 4
    oled_draw_separator(4);

    // Page 5: Current mode
    const char *mode_str = continuous_mode ? "Mode: CONT" : "Mode: CMD";
    oled_draw_string(0, 5, mode_str);
    oled_draw_string(0, 6, "Scan complete");

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

static void display_mode_change(bool continuous_mode) {
    oled_clear();
    oled_draw_separator(0);
    oled_draw_string(10, 2, "MODE CHANGED");
    if (continuous_mode) {
        oled_draw_string(4, 4, "-> CONTINUOUS");
    } else {
        oled_draw_string(10, 4, "->  COMMAND");
    }
    oled_draw_separator(6);
    oled_flush();
    sleep_ms(1500);  // Let the user see the change before returning to splash
}

// ====================================================================
// Scanner UART Functions
// ====================================================================

// Send the GM65 software trigger command over UART0
static void scanner_send_trigger(void) {
    const uint8_t cmd[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
    uart_write_blocking(SCANNER_UART, cmd, sizeof(cmd));
    printf("Trigger command sent\n");

    // Discard the 7-byte acknowledgment: 02 00 00 01 00 33 31
    uint8_t ack[8];
    int ack_received = 0;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (ack_received < 8) {
        if (uart_is_readable(SCANNER_UART)) {
            ack[ack_received++] = uart_getc(SCANNER_UART);
        }
        if (to_ms_since_boot(get_absolute_time()) - start > 2000) break;
        sleep_us(100);
    }
    printf("Acknowledgment received (%d bytes)\n", ack_received);
}

// FIX (Bug 1): The original scanner_set_continuous() and scanner_set_command()
// sent identical byte sequences (both used 0x01 in byte index 6, which means
// "trigger once" — same as scanner_send_trigger).  The mode control byte is
// index 6: 0x00 = continuous/auto scan, 0x01 = command/triggered mode.

// Configure scanner for continuous (auto) mode
static void scanner_set_continuous(void) {
    const uint8_t cmd[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x00, 0xAB, 0xCD};
    uart_write_blocking(SCANNER_UART, cmd, sizeof(cmd));
    sleep_ms(100);
    printf("Scanner set to CONTINUOUS mode\n");
}

// Configure scanner for command (triggered) mode
static void scanner_set_command(void) {
    const uint8_t cmd[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
    uart_write_blocking(SCANNER_UART, cmd, sizeof(cmd));
    sleep_ms(100);
    printf("Scanner set to COMMAND mode\n");
}

// Read a line of UART data into buf (up to max_len-1 characters).
// Stops at newline, carriage return, or when timeout_ms expires.
// Returns the number of bytes read (0 = nothing received).
//
// FIX (Warning 1): Replaced sleep_ms(1) with sleep_us(100).
// The RP2040 hardware UART FIFO is only 16 bytes deep.  At 9600 baud
// one byte arrives roughly every 1.04 ms, so sleeping 1 ms per poll
// cycle consumed almost all available time between bytes and risked
// dropping a burst payload.  sleep_us(100) yields the CPU briefly
// without surrendering an entire byte period.
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

    // ---- GPIO: Mode toggle switch (GP14) ----
    gpio_init(MODE_TOGGLE_PIN);
    gpio_set_dir(MODE_TOGGLE_PIN, GPIO_IN);
    gpio_pull_up(MODE_TOGGLE_PIN);
    // HIGH (open/floating) = Command Mode
    // LOW  (connected to GND) = Continuous Mode

    // ---- GPIO: Scan button (GP15, active LOW) ----
    gpio_init(SCAN_BUTTON_PIN);
    gpio_set_dir(SCAN_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(SCAN_BUTTON_PIN);

    // ---- Start OLED ----
    oled_init();
    printf("OLED initialized\n");

    // ---- Read initial switch position ----
    // gpio_get returns 1 (HIGH) when switch is open = Command Mode
    // gpio_get returns 0 (LOW)  when switch grounds GP14 = Continuous Mode
    bool current_mode = !gpio_get(MODE_TOGGLE_PIN); // true = Continuous
    bool last_mode    = current_mode;

    // Set scanner to match initial switch position
    if (current_mode) {
        scanner_set_continuous();
    } else {
        scanner_set_command();
    }

    printf("Initial mode: %s\n", current_mode ? "CONTINUOUS" : "COMMAND");

    // FIX (Tip 4): Removed the redundant second call to display_splash().
    // A single call is sufficient; the second one did not provide any
    // meaningful I2C health check.
    display_splash(current_mode);

    printf("Entering main loop\n");

    char scan_buf[128];
    CarData car;

    // FIX (Warning 3): Debounce state for the mode toggle switch.
    // A physical toggle bounces for ~5–20 ms; without debouncing a single
    // flip can fire 10+ mode-change events.  We require the raw pin reading
    // to be stable for MODE_DEBOUNCE_MS before accepting it as a real change.
    #define MODE_DEBOUNCE_MS 20
    uint32_t mode_stable_since = 0;
    bool     mode_raw_last     = current_mode;

    while (true) {

        // ============================================================
        // Check Mode Toggle Switch (with debounce)
        // ============================================================
        bool mode_raw = !gpio_get(MODE_TOGGLE_PIN);

        if (mode_raw != mode_raw_last) {
            // Pin changed — start or restart the stability timer
            mode_raw_last     = mode_raw;
            mode_stable_since = to_ms_since_boot(get_absolute_time());
        } else if (mode_raw != current_mode && mode_stable_since != 0) {
            // Pin has been stable since mode_stable_since; check if long enough
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - mode_stable_since >= MODE_DEBOUNCE_MS) {
                // Accept the new mode
                current_mode      = mode_raw;
                last_mode         = current_mode;
                mode_stable_since = 0;
                printf("Mode changed to: %s\n", current_mode ? "CONTINUOUS" : "COMMAND");
                if (current_mode) {
                    scanner_set_continuous();
                } else {
                    scanner_set_command();
                }
                display_mode_change(current_mode);
                display_splash(current_mode);
            }
        }

        // ============================================================
        // Command Mode — wait for button press, then trigger one scan
        // ============================================================
        if (!current_mode) {
            if (!gpio_get(SCAN_BUTTON_PIN)) {
                sleep_ms(50);  // Debounce delay
                if (!gpio_get(SCAN_BUTTON_PIN)) {
                    printf("Scan button pressed\n");
                    scanner_send_trigger();

                    // Wait for button release before reading data
                    while (!gpio_get(SCAN_BUTTON_PIN)) sleep_ms(10);

                    // Listen for scanner response — 5 second window
                    int len = scanner_readline(scan_buf, sizeof(scan_buf), 10000);
                    if (len > 0) {
                        if (parse_car_data(scan_buf, &car)) {
                            printf("Car: %s | %s | %s | %.1f T\n",
                                   car.reporting_mark, car.car_number,
                                   car.car_type, car.loaded_weight_f);
                            display_car_data(&car, false);
                        } else {
                            printf("Parse failed for: [%s]\n", scan_buf);
                            display_error("Bad QR format");
                            sleep_ms(2000);
                            display_splash(false);
                        }
                    } else {
                        printf("No data received within timeout\n");
                        display_error("No scan data");
                        sleep_ms(2000);
                        display_splash(false);
                    }
                }
            }
        }

        // ============================================================
        // Continuous Mode — poll UART and display any code that arrives
        // ============================================================
        else {
            int len = scanner_readline(scan_buf, sizeof(scan_buf), 100);
            if (len > 0) {
                if (parse_car_data(scan_buf, &car)) {
                    printf("Car: %s | %s | %s | %.1f T\n",
                           car.reporting_mark, car.car_number,
                           car.car_type, car.loaded_weight_f);
                    display_car_data(&car, true);
                } else {
                    printf("Parse failed for: [%s]\n", scan_buf);
                    display_error("Bad QR format");
                    sleep_ms(1500);
                    display_splash(true);
                }
            }
        }

        sleep_ms(10);

    } // end while(true)

    return 0;
}
