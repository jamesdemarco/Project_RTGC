#include "comms.h"
#include <string.h>

bool comms_pack_angle(const AnglePackage &angles, uint8_t *buf, size_t bufSize, size_t &outLen) {
  // Check buffer is large enough
  if (bufSize < sizeof(AnglePackage)) {
    outLen = 0;
    return false;
  }
  
  // For AVR-to-AVR communication on same platform, memcpy is simple and fast
  // For cross-platform robustness, explicit byte encoding could be used instead
  memcpy(buf, &angles, sizeof(AnglePackage));
  outLen = sizeof(AnglePackage);
  return true;
}

bool comms_unpack_angle(const uint8_t *buf, size_t len, AnglePackage &angles_out) {
  // Validate buffer size
  if (len != sizeof(AnglePackage)) {
    return false;
  }
  
  if (!buf) {
    return false;
  }
  
  // Copy buffer into struct
  memcpy(&angles_out, buf, sizeof(AnglePackage));
  return true;
}

size_t comms_get_packet_size() {
  return sizeof(AnglePackage);
}
