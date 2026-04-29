// ====================================================================
// car_data.cpp — Railroad car data parser and display functions
// ====================================================================

#include "car_data.h"
#include "oled.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

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
// Strips any leading non-alphabetic characters from reporting mark
// to handle scanner AIM ID prefix bytes.
// Returns true if all four fields were successfully found and valid.
// ====================================================================
bool parse_car_data(const char *raw, CarData *car) {
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

void display_splash(void) {
    oled_clear();
    // Pages 0-1: Title in yellow zone (large font)
    oled_draw_string_large(4, 0, "RR SCALE");
    // Pages 2-7: Instructions in blue zone
    oled_draw_separator(2);
    oled_draw_string(8,  4, "Place car on scale");
    oled_draw_string(22, 6, "Press SCAN");
    oled_flush();
}

void display_car_data(const CarData *car) {
    oled_clear();

    // Pages 0-1: Reporting mark (YELLOW zone, large font)
    oled_draw_string_large(0, 0, car->reporting_mark);

    // Pages 2-3: Car number (BLUE zone, large font)
    oled_draw_string_large(0, 2, car->car_number);

    // Pages 4-5: Car type (large font)
    oled_draw_string_large(0, 4, car->car_type);

    // Pages 6-7: Weight with OK indicator at far right (large font)
    char wt_line[16];
    snprintf(wt_line, sizeof(wt_line), "%s T", car->loaded_weight);
    oled_draw_string_large(0, 6, wt_line);
    oled_draw_string(110, 7, "OK");

    oled_flush();
}

void display_error(const char *msg) {
    oled_clear();
    // Pages 0-1: Error title in yellow zone (large font)
    oled_draw_string_large(0, 0, "ERROR");
    // Pages 2-7: Error message and retry prompt
    oled_draw_separator(2);
    oled_draw_string(0, 4, msg);
    oled_draw_string(0, 6, "Try again...");
    oled_flush();
}
