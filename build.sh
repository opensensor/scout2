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

# Create build directory
mkdir -p build
cd build

# Get the SDK import file if not present
if [ ! -f "../pico_sdk_import.cmake" ]; then
    cp $PICO_SDK_PATH/external/pico_sdk_import.cmake ..
fi

# Configure and build
cmake ..
make -j4

echo "Build complete! The UF2 file is in build/flight_controller.uf2"
