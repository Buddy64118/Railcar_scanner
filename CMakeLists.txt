cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)
# Pull in the Pico SDK (must be set before project())
# Set PICO_SDK_PATH in your environment or uncomment and edit the line below:
# set(PICO_SDK_PATH "/path/to/pico-sdk")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(railroad_scanner C CXX ASM)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# ---- Executable ---------------------------------------------
add_executable(railroad_scanner
    src/main.cpp
    src/oled.cpp
    src/scanner.cpp
    src/car_data.cpp
)
# ---- Pico SDK Libraries -------------------------------------
target_link_libraries(railroad_scanner
    pico_stdlib          # GPIO, timing, stdio
    hardware_uart        # UART for barcode scanner
    hardware_i2c         # I2C for OLED display
    hardware_gpio        # GPIO for switch and button
    pico_multicore       # (optional, included for future use)
)

# Enable USB serial output for debugging (comment out for production)
pico_enable_stdio_usb(railroad_scanner 1)
pico_enable_stdio_uart(railroad_scanner 0)

# Generate .uf2 file for drag-and-drop flashing onto the Pico W
pico_add_extra_outputs(railroad_scanner)

# Print memory usage summary after build
# (optional – remove if it clutters your output)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
