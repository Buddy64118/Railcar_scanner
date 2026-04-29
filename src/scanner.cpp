// ====================================================================
// scanner.cpp — Barcode scanner implementation
// Hardware: Waveshare 1D/2D Barcode Scanner Module
//           UART1, GP4 (TX), GP5 (RX), 9600 baud
// ====================================================================

#include "scanner.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

// ====================================================================
// Initialize UART and button GPIO
// ====================================================================
void scanner_init(void) {
    uart_init(SCANNER_UART, SCANNER_BAUD);
    gpio_set_function(SCANNER_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(SCANNER_UART_RX, GPIO_FUNC_UART);

    gpio_init(SCAN_BUTTON_PIN);
    gpio_set_dir(SCAN_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(SCAN_BUTTON_PIN);
}

// ====================================================================
// Send the software trigger command and discard the acknowledgment
// response bytes before returning so scanner_readline() sees only
// the actual barcode data.
// The scanner sends a 7-byte acknowledgment: 02 00 00 01 00 33 31
// followed immediately by the barcode data with an AIM ID prefix byte.
// We discard 7 bytes here and let parse_car_data() strip the prefix.
// ====================================================================
void scanner_send_trigger(void) {
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

// ====================================================================
// Read a line of UART data into buf (up to max_len-1 characters).
// Stops at newline, carriage return, or when timeout_ms expires.
// Returns the number of bytes read (0 = nothing received).
// Uses sleep_us(100) between polls to avoid FIFO overflow at 9600 baud.
// ====================================================================
int scanner_readline(char *buf, int max_len, uint32_t timeout_ms) {
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
