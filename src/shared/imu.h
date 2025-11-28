#pragma once

#include <stdint.h>
#include <stdbool.h>

// Initialize MPU6050 and DMP
// Returns true on success, false on failure
bool imu_begin();

// Read yaw, pitch, roll angles from DMP
// Returns true if new data was available, false otherwise
// ypr_out should be a float array of at least 3 elements: [yaw, pitch, roll]
bool imu_read_ypr(float ypr_out[3]);

// Read raw accelerometer and gyroscope values
// accel_out and gyro_out should be float arrays of at least 3 elements
// Returns true on success
bool imu_read_raw(float accel_out[3], float gyro_out[3]);

// Check if DMP is ready
bool imu_is_ready();

// Print MPU6050 status (for debugging)
void imu_print_status();
