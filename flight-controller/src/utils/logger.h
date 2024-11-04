#pragma once

#include <stdio.h>

#define LOG_DEBUG(fmt, ...) printf("DEBUG: " fmt "\n", ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) printf("INFO: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) printf("ERROR: " fmt "\n", ##__VA_ARGS__)

void logger_init(void);
void logger_flush(void);
