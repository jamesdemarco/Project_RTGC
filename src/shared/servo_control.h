#pragma once

#include "types.h"

// Initialize servos (attach to pins)
void servos_init();

// Set servo angles with limits and clipping
// Angles are in degrees
void servos_set(float angleX, float angleY);

// Lock servos to angles from AnglePackage
void servos_lock(const AnglePackage &angles);

// Set servo raw PWM values (1000-2000 microseconds typical)
// Useful for direct control if needed
void servos_set_pwm(uint16_t pwm_x, uint16_t pwm_y);
