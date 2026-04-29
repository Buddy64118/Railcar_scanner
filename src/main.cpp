// ============================================================
// Railroad Car Identification Scanner
// Hardware: Raspberry Pi Pico W
//           Waveshare Barcode Scanner Module (UART1)
//           0.96" OLED Display (I2C0, SSD1306)
//           Momentary Push Button (Scan Trigger) — GP15
//
// Connector Groups (Keyestudio Pico Breakout Board):
//   UART1 : GP4 (TX) | GP5 (RX)               -> Scanner
//   I2C0  : GP20 (SDA) | GP21 (SCL)            -> OLED
//
// QR Code Data Format (pipe-delimited, no spaces):
//   REPORTING_MARK|CAR_NUMBER|CAR_TYPE|LOADED_WEIGHT
//   Example: BNSF|123456|Boxcar|47.3
// ============================================================

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "oled.h"
#include "scanner.h"
#include "car_data.h"

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

    // ---- UART and button init for scanner ----
    scanner_init();
    printf("UART1 initialized on GP%d (TX) and GP%d (RX) at %d baud\n",
           SCANNER_UART_TX, SCANNER_UART_RX, SCANNER_BAUD);

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
                int len = scanner_readline(scan_buf, sizeof(scan_buf),
                                           SCAN_TIMEOUT_MS);
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
