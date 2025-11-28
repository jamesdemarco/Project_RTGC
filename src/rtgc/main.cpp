#include <Arduino.h>
#include <Wire.h>

// Shared headers
#include "../shared/config.h"
#include "../shared/types.h"
#include "../shared/radio.h"
#include "../shared/imu.h"
#include "../shared/comms.h"
#include "../shared/utils.h"

// ============ State Machine ============
enum State { IDLE, TRANSMITTING, LOCKED };
State currentState = IDLE;

// ============ Pressure Sensor Pattern Detection ============
unsigned long lastPressTime = 0;
unsigned long patternStartTime = 0;
unsigned long timeStamps[3] = {0, 0, 0};
int signalCount = 0;
bool waitingForPattern = false;
bool lastPressureState = LOW;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);  // Allow serial to stabilize
  
  Serial.println(F("RTGC (Remote Transmitter) Board Starting..."));
  
  // Initialize pressure sensor
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  Serial.println(F("Pressure sensor initialized"));
  
  // Initialize radio (sender mode)
  if (!radio_init_sender()) {
    Serial.println(F("ERROR: Radio initialization failed"));
    while (1);  // Hang
  }
  Serial.println(F("Radio initialized (sender mode)"));
  
  // Initialize IMU
  Wire.begin();
  if (!imu_begin()) {
    Serial.println(F("ERROR: IMU initialization failed"));
    while (1);  // Hang
  }
  Serial.println(F("IMU initialized"));
  
  Serial.println(F("Setup complete!"));
}

void loop() {
  // Always monitor the pressure sensor
  monitorPressureSensor();

  // Handle current state
  switch (currentState) {
    case IDLE:
      Serial.println(F("State: IDLE"));
      break;
      
    case TRANSMITTING:
      transmitIMUData();
      break;
      
    case LOCKED:
      sendOrientationLockMessage();
      break;
  }
}
 

// ============ Pressure Sensor Monitoring ============
void monitorPressureSensor() {
  int sensorValue = analogRead(PRESSURE_SENSOR_PIN);
  bool currentPressureState;

  // Determine HIGH/LOW based on thresholds
  if (sensorValue > PRESSURE_THRESHOLD_HIGH) {
    currentPressureState = HIGH;
  } else if (sensorValue < PRESSURE_THRESHOLD_LOW) {
    currentPressureState = LOW;
  } else {
    return;  // Ignore ambiguous range
  }

  // Detect rising edge (LOW->HIGH transition)
  if (currentPressureState == HIGH && lastPressureState == LOW && 
      millis() - lastPressTime > PRESSURE_DEBOUNCE_MS) {
    lastPressTime = millis();
    
    Serial.println(F("Valid pressure pulse detected"));

    if (!waitingForPattern) {
      // Start new pattern detection window
      waitingForPattern = true;
      patternStartTime = millis();
      signalCount = 0;
      Serial.println(F("Pattern detection window opened"));
    }

    shiftAndStore(millis());
    signalCount++;
    Serial.print(F("Signal count: "));
    Serial.println(signalCount);
  }

  lastPressureState = currentPressureState;

  // Check if pattern detection window has closed
  if (waitingForPattern && millis() - patternStartTime > PRESSURE_TIME_WINDOW_MS) {
    decideState();
    resetPattern();
  }
}

// ============ State Decision Logic ============
void decideState() {
  switch (signalCount) {
    case 1:
      Serial.println(F("Pattern: 1 pulse -> TRANSMITTING"));
      currentState = TRANSMITTING;
      break;
      
    case 2:
      Serial.println(F("Pattern: 2 pulses -> IDLE"));
      currentState = IDLE;
      break;
      
    case 3:
      Serial.println(F("Pattern: 3 pulses -> LOCKED"));
      currentState = LOCKED;
      break;
      
    default:
      Serial.println(F("Pattern: invalid -> IDLE"));
      currentState = IDLE;
      break;
  }
}

// ============ IMU Data Transmission ============
void transmitIMUData() {
  if (!imu_is_ready()) {
    return;
  }

  float ypr[3];
  if (imu_read_ypr(ypr)) {
    // ypr[0] = yaw, ypr[1] = pitch, ypr[2] = roll
    // Convert from radians to degrees with offset
    float angleX = (SERVO_X_CENTER + (ypr[1] * 180.0f / M_PI));
    float angleY = (SERVO_Y_CENTER - (ypr[2] * 180.0f / M_PI));

    // Create and pack angle package
    AnglePackage angles;
    angles.msg_id = MSG_ANGLE;
    angles.angleX = angleX;
    angles.angleY = angleY;
    angles.timestamp = millis();
    angles.isLocked = false;

    // Send via radio
    uint8_t buf[32];
    size_t bufLen = 0;
    if (comms_pack_angle(angles, buf, sizeof(buf), bufLen)) {
      radio_send(buf, bufLen);
      Serial.print(F("TX: X="));
      Serial.print(angleX);
      Serial.print(F(" Y="));
      Serial.println(angleY);
    }
  }
}

// ============ Orientation Lock ============
void sendOrientationLockMessage() {
  if (!imu_is_ready()) {
    return;
  }

  float ypr[3];
  if (imu_read_ypr(ypr)) {
    // Gather current orientation
    float angleX = (SERVO_X_CENTER + (ypr[1] * 180.0f / M_PI));
    float angleY = (SERVO_Y_CENTER - (ypr[2] * 180.0f / M_PI));

    // Create lock angle package
    AnglePackage lockAngles;
    lockAngles.msg_id = MSG_LOCK;
    lockAngles.angleX = angleX;
    lockAngles.angleY = angleY;
    lockAngles.isLocked = true;
    lockAngles.timestamp = millis();

    // Send via radio
    uint8_t buf[32];
    size_t bufLen = 0;
    if (comms_pack_angle(lockAngles, buf, sizeof(buf), bufLen)) {
      radio_send(buf, bufLen);
      Serial.print(F("LOCK TX: X="));
      Serial.print(angleX);
      Serial.print(F(" Y="));
      Serial.println(angleY);
    }
  }

  // Return to IDLE after sending lock
  currentState = IDLE;
}

// ============ Helper Functions ============
void shiftAndStore(unsigned long currentTime) {
  for (int i = 2; i > 0; i--) {
    timeStamps[i] = timeStamps[i - 1];
  }
  timeStamps[0] = currentTime;
}

void resetPattern() {
  signalCount = 0;
  waitingForPattern = false;
  for (int i = 0; i < 3; i++) {
    timeStamps[i] = 0;
  }
}
