#!/bin/bash

# Save as build.sh and make executable with: chmod +x build.sh

# Check if pico-sdk is set
if [ -z "${PICO_SDK_PATH}" ]; then
    echo "PICO_SDK_PATH is not set. Setting it up..."
    
    # Clone Pico SDK if not already present
    if [ ! -d "pico-sdk" ]; then
        git clone https://github.com/raspberrypi/pico-sdk.git
        cd pico-sdk
        git submodule update --init
        cd ..
    fi
    
    # Export SDK path
    export PICO_SDK_PATH=$(pwd)/pico-sdk
fi

# Create build directory if it doesn't exist
mkdir -p build

# Get the SDK import file if not present
if [ ! -f "pico_sdk_import.cmake" ]; then
    cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
fi

# Create CMakeLists.txt if it doesn't exist
if [ ! -f "CMakeLists.txt" ]; then
cat > CMakeLists.txt << 'EOL'
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
    flight-controller/src/main.c
    flight-controller/src/core/flight_controller.c
    flight-controller/src/core/attitude_estimator.c
    flight-controller/src/core/pid_controller.c
    flight-controller/src/drivers/mpu6050.c
    flight-controller/src/drivers/esc.c
)

# Add pico_stdlib which pulls in commonly used features
target_link_libraries(flight_controller 
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_timer
    pico_multicore
)

# Create map/bin/hex file etc.
pico_add_extra_outputs(flight_controller)

# Enable USB output, disable UART output
pico_enable_stdio_usb(flight_controller 1)
pico_enable_stdio_uart(flight_controller 0)
EOL
fi

# Go to build directory and run cmake
cd build
cmake ..
make -j4

echo "Build complete! The UF2 file is in build/flight_controller.uf2"
