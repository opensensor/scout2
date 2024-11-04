#include "logger.h"

void logger_init(void) {
    // Initialize stdio (already done in system_init)
}

void logger_flush(void) {
    // Force output buffer flush
    stdio_flush();
}
