cmake_minimum_required(VERSION 3.13)

# Command line option to control what we're building
option(BUILD_HOST "Build host tests" ON)
option(BUILD_PICO "Build Pico targets" ON)

if(BUILD_HOST)
    # Host executable (using system compiler)
    project(flight_controller_host C CXX)
    
    add_executable(flight_controller_tests_host
        flight-controller/tests/test_main.c
        flight-controller/tests/test_framework.c
        flight-controller/tests/pid_controller_tests.c
        flight-controller/tests/attitude_estimator_tests.c
        flight-controller/src/core/pid_controller.c
        flight-controller/src/core/attitude_estimator.c
    )

    target_include_directories(flight_controller_tests_host PRIVATE
        ${CMAKE_SOURCE_DIR}/flight-controller/src
        ${CMAKE_SOURCE_DIR}/flight-controller/src/include
        ${CMAKE_SOURCE_DIR}/flight-controller/src/core
        ${CMAKE_SOURCE_DIR}/flight-controller/src/drivers
        ${CMAKE_SOURCE_DIR}/flight-controller/src/utils
        ${CMAKE_SOURCE_DIR}/flight-controller/tests
    )

    target_compile_definitions(flight_controller_tests_host PRIVATE 
        HOST_BUILD=1
    )

    target_link_libraries(flight_controller_tests_host m)

    # Add custom target to run host tests
    add_custom_target(run_tests
        COMMAND ./flight_controller_tests_host
        DEPENDS flight_controller_tests_host
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endif()

if(BUILD_PICO)
    # Pico targets
    include(pico_sdk_import.cmake)
    project(flight_controller C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    pico_sdk_init()

    # Common include directories for Pico targets
    set(COMMON_INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/flight-controller/src
        ${CMAKE_SOURCE_DIR}/flight-controller/src/include
        ${CMAKE_SOURCE_DIR}/flight-controller/src/core
        ${CMAKE_SOURCE_DIR}/flight-controller/src/drivers
        ${CMAKE_SOURCE_DIR}/flight-controller/src/utils
        ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
        ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
        ${PICO_SDK_PATH}/src/rp2_common/hardware_i2c/include
        ${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include
    )

    # Common test source files
    set(TEST_SOURCES
        flight-controller/tests/test_main.c
        flight-controller/tests/test_framework.c
        flight-controller/tests/pid_controller_tests.c
        flight-controller/tests/attitude_estimator_tests.c
	flight-controller/tests/mpu6050_tests.c
        flight-controller/src/core/pid_controller.c
        flight-controller/src/core/attitude_estimator.c
	flight-controller/src/drivers/mpu6050.c
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
    add_executable(flight_controller_tests_pico ${TEST_SOURCES})

    target_include_directories(flight_controller_tests_pico PRIVATE
        ${COMMON_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/flight-controller/tests
    )

    target_link_libraries(flight_controller_tests_pico
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_timer
        pico_runtime
        m
    )

    pico_enable_stdio_usb(flight_controller_tests_pico 1)
    pico_enable_stdio_uart(flight_controller_tests_pico 0)
    pico_add_extra_outputs(flight_controller_tests_pico)
endif()
