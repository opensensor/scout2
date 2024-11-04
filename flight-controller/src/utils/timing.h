#pragma once

#include "hardware/timer.h"
#include "pico/stdlib.h"

#define DT 0.002f  // 500Hz update rate = 2ms period

static inline uint64_t get_time_us(void) {
    return time_us_64();
}

static inline void delay_us(uint32_t us) {
    sleep_us(us);
}
