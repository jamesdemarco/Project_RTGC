#include "servo_control.h"
#include "config.h"
#include "utils.h"
#include <Servo.h>

// Global servo instances
static Servo servo_x;
static Servo servo_y;

void servos_init() {
  servo_x.attach(SERVO_X_PIN);
  servo_y.attach(SERVO_Y_PIN);
  
  // Set servos to center position on startup
  servo_x.write(SERVO_X_CENTER);
  servo_y.write(SERVO_Y_CENTER);
}

void servos_set(float angleX, float angleY) {
  // Clamp angles to valid servo range
  float clampedX = clamp(angleX, (float)SERVO_X_MIN, (float)SERVO_X_MAX);
  float clampedY = clamp(angleY, (float)SERVO_Y_MIN, (float)SERVO_Y_MAX);
  
  // Write to servos (Servo::write expects 0-180 degrees)
  servo_x.write((uint8_t)clampedX);
  servo_y.write((uint8_t)clampedY);
}

void servos_lock(const AnglePackage &angles) {
  servos_set(angles.angleX, angles.angleY);
}

void servos_set_pwm(uint16_t pwm_x, uint16_t pwm_y) {
  servo_x.writeMicroseconds(pwm_x);
  servo_y.writeMicroseconds(pwm_y);
}
