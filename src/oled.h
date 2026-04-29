// ====================================================================
// oled.h — OLED display function declarations
// Hardware: 0.96" SSD1306 OLED, I2C0, GP20 (SDA), GP21 (SCL)
// ====================================================================

#pragma once

#include <stdint.h>

// ---- Pin and I2C Definitions --------------------------------
#define OLED_I2C            i2c0
#define OLED_SDA_PIN        20
#define OLED_SCL_PIN        21
#define OLED_I2C_ADDR       0x3C
#define OLED_I2C_FREQ       400000

// I2C timeout values (microseconds)
#define I2C_CMD_TIMEOUT_US  10000
#define I2C_DATA_TIMEOUT_US 25000

// ---- Display Dimensions -------------------------------------
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGES          8

// ---- Tunable Parameters -------------------------------------
#define OLED_CONTRAST       0xCF    // Display brightness 0x00-0xFF

// ---- Function Declarations ----------------------------------

// Initialization
void oled_init(void);

// Frame buffer
void oled_clear(void);
void oled_flush(void);

// Drawing primitives
void oled_draw_char(uint8_t x, uint8_t page, char c);
void oled_draw_string(uint8_t x, uint8_t page, const char *str);
void oled_draw_char_large(uint8_t x, uint8_t page, char c);
void oled_draw_string_large(uint8_t x, uint8_t page, const char *str);
void oled_draw_separator(uint8_t page);
