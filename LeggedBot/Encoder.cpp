/*
   @filename Encoder.h
   @author TEAM TEN
   @version 1.0
   @brief Stripped down version of the AMS_AS5048B library. (we ran out of memory...)

   This could potentially just be merged into LegController.
*/

#include <Wire.h>
#include <stdint.h>
#include "Encoder.h"

/**
   @brief Read the encoder angle from the I2C bus.
   @return 14-bit angle
*/
uint16_t Encoder::read() {
  Wire.beginTransmission(address);
  Wire.write(0xFE); // angle register
  Wire.endTransmission(false);

  Wire.requestFrom(address, 2);

  return (((uint16_t)Wire.read()) << 6) | (Wire.read() & 0x3F);
}

