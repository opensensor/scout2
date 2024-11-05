#!/bin/bash
# build.sh

# Function to display usage
usage() {
    echo "Usage: $0 [host|pico|all]"
    echo "  host: Build and run host tests"
    echo "  pico: Build Pico targets"
    echo "  all:  Build both (in separate directories)"
    exit 1
}

# Create build directories if they don't exist
mkdir -p build-host
mkdir -p build-pico

case "$1" in
    "host")
        echo "Building host tests..."
        cd build-host
        cmake -DBUILD_PICO=OFF ..
        make
        ./flight_controller_tests_host
        ;;
    "pico")
        echo "Building Pico targets..."
        cd build-pico
        cmake -DBUILD_HOST=OFF -DPICO_BOARD=pico2 ..
        make
        ;;
    "all")
        echo "Building host tests..."
        cd build-host
        cmake -DBUILD_PICO=OFF ..
        make
        ./flight_controller_tests_host
        
        cd ..
        
        echo "Building Pico targets..."
        cd build-pico
        cmake -DBUILD_HOST=OFF ..
        make
        ;;
    *)
        usage
        ;;
esac
