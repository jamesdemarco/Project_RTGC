#pragma once

#include <stddef.h>
#include <stdint.h>

// Initialize radio as a sender (write-only mode)
bool radio_init_sender();

// Initialize radio as a listener (read-only mode)
bool radio_init_listener();

// Send a packet of data
// Returns true on successful transmission
bool radio_send(const uint8_t *buf, size_t len);

// Check if data is available to read
bool radio_available();

// Read incoming packet data
// Returns number of bytes read, 0 if no data available
size_t radio_read(uint8_t *buf, size_t bufSize);

// Get the current operating status (for debugging)
void radio_print_status();
