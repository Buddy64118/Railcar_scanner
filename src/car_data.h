// ====================================================================
// car_data.h — Railroad car data structure and function declarations
// ====================================================================

#pragma once

#include <stdbool.h>

// ====================================================================
// Car Data Structure
// ====================================================================
typedef struct {
    char  reporting_mark[16];   // Railroad reporting mark e.g. "BNSF"
    char  car_number[12];       // Car number e.g. "123456"
    char  car_type[24];         // Car type e.g. "Boxcar"
    char  loaded_weight[12];    // Weight as display string e.g. "47.3"
    float loaded_weight_f;      // Weight as float for validation
} CarData;

// ====================================================================
// Function Declarations
// ====================================================================

// Parse a pipe-delimited QR code string into a CarData struct.
// Returns true if all four fields were found and weight is valid.
bool parse_car_data(const char *raw, CarData *car);

// Display functions
void display_splash(void);
void display_car_data(const CarData *car);
void display_error(const char *msg);
