#pragma once

#include <stdint.h>

// ============ RF24 Radio Configuration ============
#define RADIO_CE_PIN 9
#define RADIO_CSN_PIN 10
#define RADIO_ADDRESS "00001"
#define RADIO_PA_LEVEL RF24_PA_MIN

// ============ IMU / MPU6050 Configuration ============
#define IMU_INTERRUPT_PIN 2

// MPU6050 calibration offsets (base gimbal calibration)
#define MPU_OFFSET_X_GYRO -223
#define MPU_OFFSET_Y_GYRO 17
#define MPU_OFFSET_Z_GYRO 50
#define MPU_OFFSET_X_ACCEL 2569
#define MPU_OFFSET_Y_ACCEL -2470
#define MPU_OFFSET_Z_ACCEL 1063

// ============ Servo Configuration ============
#define SERVO_X_PIN 3
#define SERVO_Y_PIN 5  // Adjust if needed; original used pin 2 but that's also INTERRUPT_PIN

// Servo angle limits (base gimbal)
#define SERVO_X_MIN 20
#define SERVO_X_MAX 180
#define SERVO_Y_MIN 5
#define SERVO_Y_MAX 143

// Servo center/default angles
#define SERVO_X_CENTER 95
#define SERVO_Y_CENTER 115

// ============ Wireless Communication ============
#define WIRELESS_TIMEOUT_MS 1000

// ============ Pressure Sensor (RTGC) ============
#define PRESSURE_SENSOR_PIN A2
#define PRESSURE_THRESHOLD_HIGH 300
#define PRESSURE_THRESHOLD_LOW 50
#define PRESSURE_DEBOUNCE_MS 200
#define PRESSURE_TIME_WINDOW_MS 1000

// ============ Serial Configuration ============
#define SERIAL_BAUD 115200
