cmake_minimum_required(VERSION 3.13)

# Command line option to control what we're building
option(BUILD_HOST "Build host tests" ON)
option(BUILD_PICO "Build Pico targets" ON)

if(BUILD_PICO)
    # Pico SDK must be initialized first
    include(pico_sdk_import.cmake)
    project(flight_controller C CXX ASM)
    pico_sdk_init()
endif()

# Create unity config directory
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/unity_config)
configure_file(
    ${CMAKE_SOURCE_DIR}/flight-controller/tests/unity_config.h
    ${CMAKE_BINARY_DIR}/unity_config/unity_config.h
    COPYONLY
)

# Set up Unity library
add_library(unity STATIC
    third_party/unity/src/unity.c
)

if(BUILD_PICO)
    # For Pico target, we need to set specific compiler flags for Unity
    target_compile_definitions(unity PUBLIC
        UNITY_INCLUDE_CONFIG_H
        UNITY_OUTPUT_COLOR
    )
    
    # Link Unity with Pico SDK
    target_link_libraries(unity PRIVATE
        pico_stdlib
    )
endif()

target_include_directories(unity PUBLIC 
    third_party/unity/src
    ${CMAKE_BINARY_DIR}/unity_config
)

if(BUILD_HOST)
    project(flight_controller_host C CXX)
    # ... (host build configuration remains the same)
endif()

if(BUILD_PICO)
    # Common include directories for Pico targets
    set(COMMON_INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/flight-controller/src
        ${CMAKE_SOURCE_DIR}/flight-controller/src/include
        ${CMAKE_SOURCE_DIR}/flight-controller/src/core
        ${CMAKE_SOURCE_DIR}/flight-controller/src/drivers
        ${CMAKE_SOURCE_DIR}/flight-controller/src/utils
        ${CMAKE_SOURCE_DIR}/third_party/unity/src
        ${CMAKE_BINARY_DIR}/unity_config
        ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
        ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
        ${PICO_SDK_PATH}/src/rp2_common/hardware_i2c/include
        ${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include
    )

    # Main flight controller executable
    add_executable(flight_controller
        flight-controller/src/main.c
        flight-controller/src/core/flight_controller.c
        flight-controller/src/core/attitude_estimator.c
        flight-controller/src/core/pid_controller.c
        flight-controller/src/drivers/mpu6050.c
        flight-controller/src/drivers/esc.c
        flight-controller/src/drivers/system.c
        flight-controller/src/utils/logger.c
    )

    target_include_directories(flight_controller PRIVATE ${COMMON_INCLUDE_DIRS})
    
    target_link_libraries(flight_controller 
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_timer
        pico_multicore
    )

    pico_add_extra_outputs(flight_controller)
    pico_enable_stdio_usb(flight_controller 1)
    pico_enable_stdio_uart(flight_controller 0)

    # Pico test executable
    add_executable(flight_controller_tests_pico
        flight-controller/tests/test_main.c
        flight-controller/tests/pid_controller_tests.c
        flight-controller/tests/attitude_estimator_tests.c
        flight-controller/tests/mpu6050_tests.c
        flight-controller/src/core/pid_controller.c
        flight-controller/src/core/attitude_estimator.c
        flight-controller/src/drivers/mpu6050.c
    )

    target_include_directories(flight_controller_tests_pico PRIVATE ${COMMON_INCLUDE_DIRS})

    target_compile_definitions(flight_controller_tests_pico PRIVATE
        PICO_STDIO_USB=1
    )

    target_link_libraries(flight_controller_tests_pico
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_timer
        pico_runtime
        unity
        m
    )

    pico_enable_stdio_usb(flight_controller_tests_pico 1)
    pico_enable_stdio_uart(flight_controller_tests_pico 0)
    pico_add_extra_outputs(flight_controller_tests_pico)
endif()
