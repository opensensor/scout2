#include "core/flight_controller.h"
#include "drivers/system.h"
#include "utils/timing.h"
#include "utils/logger.h"
#include <stdio.h>

int main(void) {
    // Initialize system
    if (system_init() != 0) {
        printf("System initialization failed!\n");
        return -1;
    }

    // Initialize flight controller
    flight_controller_t* fc = flight_controller_init();
    if (fc == NULL) {
        printf("Flight controller initialization failed!\n");
        return -1;
    }

    // Main loop
    while (1) {
        uint64_t loop_start = get_time_us();
        
        // Update flight controller
        flight_controller_update(fc);
        
        // Maintain loop timing
        uint64_t elapsed = get_time_us() - loop_start;
        if (elapsed < CONTROL_LOOP_PERIOD_US) {
            delay_us(CONTROL_LOOP_PERIOD_US - elapsed);
        }
    }

    // Cleanup (never reached in normal operation)
    flight_controller_cleanup(fc);
    return 0;
}
