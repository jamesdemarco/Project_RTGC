#pragma once

#include "types.h"
#include <stddef.h>
#include <stdint.h>

// Pack an AnglePackage struct into a byte buffer
// Returns true on success, false if buffer is too small
bool comms_pack_angle(const AnglePackage &angles, uint8_t *buf, size_t bufSize, size_t &outLen);

// Unpack a byte buffer into an AnglePackage struct
// Returns true on success, false if buffer is invalid or too small
bool comms_unpack_angle(const uint8_t *buf, size_t len, AnglePackage &angles_out);

// Get the expected size for an AnglePackage when packed
// (Useful for validation)
size_t comms_get_packet_size();
