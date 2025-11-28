#include <Arduino.h>
#include <Wire.h>

// Shared headers
#include "../shared/config.h"
#include "../shared/types.h"
#include "../shared/radio.h"
#include "../shared/imu.h"
#include "../shared/servo_control.h"
#include "../shared/comms.h"
#include "../shared/utils.h"

// ============ State Machine ============
enum State { USING_IMU, USING_WIRELESS };
State currentState = USING_IMU;

// ============ Timing ============
unsigned long lastWirelessTime = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);  // Allow serial to stabilize
  
  Serial.println(F("Base Gimbal Board Starting..."));
  
  // Initialize servos
  servos_init();
  Serial.println(F("Servos initialized"));
  
  // Initialize radio (listener mode)
  if (!radio_init_listener()) {
    Serial.println(F("ERROR: Radio initialization failed"));
    while (1);  // Hang
  }
  Serial.println(F("Radio initialized (listener mode)"));
  
  // Initialize IMU
  Wire.begin();
  if (!imu_begin()) {
    Serial.println(F("ERROR: IMU initialization failed"));
    while (1);  // Hang
  }
  Serial.println(F("IMU initialized"));
  
  lastWirelessTime = millis();
  Serial.println(F("Setup complete!"));
}

void loop() {
  if (!imu_is_ready()) {
    return;
  }

  // Check for incoming wireless commands
  if (radio_available()) {
    uint8_t buf[32];
    size_t len = radio_read(buf, sizeof(buf));
    
    if (len > 0) {
      AnglePackage angles;
      if (comms_unpack_angle(buf, len, angles)) {
        lastWirelessTime = millis();
        
        // Check for lock command
        if (angles.isLocked) {
          Serial.println(F("Lock command received"));
          servos_lock(angles);
          currentState = USING_WIRELESS;
          return;  // Skip other updates
        }
        
        // Regular angle update
        currentState = USING_WIRELESS;
        Serial.print(F("Wireless: X="));
        Serial.print(angles.angleX);
        Serial.print(F(" Y="));
        Serial.println(angles.angleY);
      }
    }
  }

  // Check for wireless timeout
  if (millis() - lastWirelessTime > WIRELESS_TIMEOUT_MS) {
    currentState = USING_IMU;
  }

  // Execute based on current state
  if (currentState == USING_WIRELESS) {
    // Angles will be held from last wireless command
    // (servos already set in the radio_available block above)
  } else {
    // Use IMU for gimbal control
    float ypr[3];
    if (imu_read_ypr(ypr)) {
      // ypr[0] = yaw, ypr[1] = pitch, ypr[2] = roll
      // Convert from radians to degrees and apply offset/limits
      float angle1 = (SERVO_X_CENTER - ypr[2] * 180.0f / M_PI);
      float angle2 = (SERVO_Y_CENTER - ypr[1] * 180.0f / M_PI);
      
      servos_set(angle1, angle2);
      
      Serial.print(F("IMU: X="));
      Serial.print(angle1);
      Serial.print(F(" Y="));
      Serial.println(angle2);
    }
  }
}
