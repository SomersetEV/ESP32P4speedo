#pragma once
#include <stdint.h>
#include <stdbool.h>

void odometer_init(void);
void odometer_add_meters(uint32_t meters);
void odometer_periodic_save(void);
void odometer_force_save(void);
void odometer_reset(void);

uint64_t odometer_get_meters(void);
double odometer_get_miles(void);