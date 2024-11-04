#include "core/flight_controller.h"
#include "drivers/system.h"
#include "include/config.h"
#include "utils/logger.h"
#include <stdio.h>

int main(void) {
    if (system_init() != 0) {
        printf("System initialization failed!\n");
        return -1;
    }

    flight_controller_t* fc = flight_controller_init();
    if (fc == NULL) {
        printf("Flight controller initialization failed!\n");
        return -1;
    }

    uint64_t last_update = time_us_64();
    const uint64_t update_period_us = (uint64_t)(DT * 1000000.0f);

    while (1) {
        uint64_t now = time_us_64();
        if (now - last_update >= update_period_us) {
            flight_controller_update(fc);
            last_update = now;
        }
        sleep_us(100); // Small delay to prevent tight spinning
    }

    return 0;
}
