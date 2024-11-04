cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(flight_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(
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
