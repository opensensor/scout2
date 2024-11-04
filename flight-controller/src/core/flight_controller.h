#pragma once

#include "types.h"
#include "attitude_estimator.h"
#include "pid_controller.h"

typedef struct flight_controller flight_controller_t;

flight_controller_t* flight_controller_init(void);
void flight_controller_update(flight_controller_t* fc);
void flight_controller_cleanup(flight_controller_t* fc);
