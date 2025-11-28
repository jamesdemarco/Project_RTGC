#pragma once

#include <stdint.h>

// Message ID enumeration
enum MsgID : uint8_t {
  MSG_ANGLE = 1,
  MSG_LOCK = 2
};

// Shared data structure for angle transmission
struct AnglePackage {
  uint8_t msg_id;       // Message type (MSG_ANGLE or MSG_LOCK)
  float angleX;         // X rotation angle in degrees
  float angleY;         // Y rotation angle in degrees
  uint32_t timestamp;   // Timestamp in milliseconds
  uint8_t isLocked;     // 1 = lock command, 0 = normal angle update
};

// Protocol constants
constexpr uint16_t ANGLE_PACKAGE_SIZE = sizeof(AnglePackage);
