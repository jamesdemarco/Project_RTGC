#include "imu.h"
#include "config.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// Global MPU6050 instance
static MPU6050 mpu;

// DMP state variables
static bool dmpReady = false;
static uint8_t mpuIntStatus;
static uint16_t packetSize;
static uint8_t fifoBuffer[64];

// Quaternion and gravity for angle calculation
static Quaternion q;
static VectorFloat gravity;

// ISR flag
static volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

bool imu_begin() {
  // Initialize I2C and MPU6050
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    return false;
  }
  
  // Initialize DMP
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    return false;
  }
  
  // Apply calibration offsets
  mpu.setXGyroOffset(MPU_OFFSET_X_GYRO);
  mpu.setYGyroOffset(MPU_OFFSET_Y_GYRO);
  mpu.setZGyroOffset(MPU_OFFSET_Z_GYRO);
  mpu.setXAccelOffset(MPU_OFFSET_X_ACCEL);
  mpu.setYAccelOffset(MPU_OFFSET_Y_ACCEL);
  mpu.setZAccelOffset(MPU_OFFSET_Z_ACCEL);
  
  // Calibrate
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  
  // Enable DMP
  mpu.setDMPEnabled(true);
  
  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), dmpDataReady, RISING);
  
  // Get initial status and packet size
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;
  
  return true;
}

bool imu_read_ypr(float ypr_out[3]) {
  if (!dmpReady) {
    return false;
  }
  
  if (!mpuInterrupt && mpu.getFIFOCount() < packetSize) {
    return false;
  }
  
  // Reset interrupt flag
  mpuInterrupt = false;
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr_out, &q, &gravity);
    return true;
  }
  
  return false;
}

bool imu_read_raw(float accel_out[3], float gyro_out[3]) {
  int16_t rawAccX, rawAccY, rawAccZ;
  int16_t rawGyroX, rawGyroY, rawGyroZ;
  
  mpu.getMotion6(&rawAccX, &rawAccY, &rawAccZ, &rawGyroX, &rawGyroY, &rawGyroZ);
  
  // Convert to float (accelerometer: 16384 LSB/g, gyroscope: 131 LSB/(deg/s))
  accel_out[0] = rawAccX / 16384.0f;
  accel_out[1] = rawAccY / 16384.0f;
  accel_out[2] = rawAccZ / 16384.0f;
  
  gyro_out[0] = rawGyroX / 131.0f;
  gyro_out[1] = rawGyroY / 131.0f;
  gyro_out[2] = rawGyroZ / 131.0f;
  
  return true;
}

bool imu_is_ready() {
  return dmpReady;
}

void imu_print_status() {
  Serial.print("DMP ready: ");
  Serial.print(dmpReady);
  Serial.print(", FIFO count: ");
  Serial.print(mpu.getFIFOCount());
  Serial.print(", Packet size: ");
  Serial.println(packetSize);
}
