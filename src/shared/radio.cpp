#include "radio.h"
#include "config.h"
#include <RF24.h>

// Global RF24 instance (instantiated once)
static RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

bool radio_init_sender() {
  if (!radio.begin()) {
    return false;
  }
  radio.openWritingPipe((const uint8_t *)RADIO_ADDRESS);
  radio.setPALevel(RADIO_PA_LEVEL);
  radio.stopListening();  // TX mode
  return true;
}

bool radio_init_listener() {
  if (!radio.begin()) {
    return false;
  }
  radio.openReadingPipe(0, (const uint8_t *)RADIO_ADDRESS);
  radio.setPALevel(RADIO_PA_LEVEL);
  radio.startListening();  // RX mode
  return true;
}

bool radio_send(const uint8_t *buf, size_t len) {
  if (!buf || len == 0) {
    return false;
  }
  return radio.write(buf, len);
}

bool radio_available() {
  return radio.available();
}

size_t radio_read(uint8_t *buf, size_t bufSize) {
  if (!buf || bufSize == 0) {
    return 0;
  }
  
  if (!radio.available()) {
    return 0;
  }
  
  // Get the actual payload size
  uint8_t payload_size = radio.getDynamicPayloadSize();
  
  if (payload_size > bufSize) {
    payload_size = bufSize;
  }
  
  radio.read(buf, payload_size);
  return payload_size;
}

void radio_print_status() {
  radio.printDetails();
}
