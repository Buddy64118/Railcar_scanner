// ====================================================================
// scanner.h — Barcode scanner function declarations
// Hardware: Waveshare 1D/2D Barcode Scanner Module
//           UART1, GP4 (TX), GP5 (RX), 9600 baud
// ====================================================================

#pragma once

#include <stdint.h>

// ---- Pin and UART Definitions -------------------------------
#define SCANNER_UART        uart1
#define SCANNER_UART_TX     4       // GP4 → Scanner RX
#define SCANNER_UART_RX     5       // GP5 ← Scanner TX
#define SCANNER_BAUD        9600

// ---- Tunable Parameters -------------------------------------
#define SCAN_TIMEOUT_MS     10000   // How long to wait for scanner response (ms)
#define SCAN_DEBOUNCE_MS    50      // Button debounce delay (ms)
#define SCAN_ERROR_HOLD_MS  2000    // How long error screen stays visible (ms)

// ---- GPIO ---------------------------------------------------
#define SCAN_BUTTON_PIN     15      // GP15 — Momentary button, active LOW

// ---- Function Declarations ----------------------------------
void scanner_init(void);
void scanner_send_trigger(void);
int  scanner_readline(char *buf, int max_len, uint32_t timeout_ms);
