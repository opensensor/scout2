cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(flight_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(flight_controller
        src/main.c
        src/core/flight_controller.c
        src/core/attitude_estimator.c
        src/core/pid_controller.c
        src/drivers/mpu6050.c
        src/drivers/esc.c
)

# Add pico_stdlib which pulls in commonly used features
target_link_libraries(flight_controller
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_timer
        pico_multicore  # If you want to use both cores
)

# Create map/bin/hex file etc.
pico_add_extra_outputs(flight_controller)

# Enable USB output, disable UART output
pico_enable_stdio_usb(flight_controller 1)
pico_enable_stdio_uart(flight_controller 0)
