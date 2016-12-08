#include <Wire.h>
#include <stdint.h>
#include "Encoder.h"

Encoder::Encoder(uint8_t address) : address_(address) {}

/**
   @brief Read the encoder angle from the I2C bus.
   @return 14-bit angle
*/
uint16_t Encoder::read() {
  Wire.beginTransmission(address_);
  Wire.write(0xFE); // angle register
  Wire.endTransmission(false);

  Wire.requestFrom(address_, 2);

  return ((static_cast<uint16_t>(Wire.read())) << 6) | (Wire.read() & 0x3F);
}

