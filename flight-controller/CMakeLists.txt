cmake_minimum_required(VERSION 3.12)
project(flight_controller C)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add compilation flags
add_compile_options(-Wall -Wextra -O2)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

# Collect source files
file(GLOB_RECURSE SOURCES 
    "src/*.c"
)

# Create executable
add_executable(flight_controller ${SOURCES})

# Link libraries
target_link_libraries(flight_controller m)  # Link math library

# Add tests
enable_testing()
add_subdirectory(tests)
