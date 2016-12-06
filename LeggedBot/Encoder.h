/*
   @filename Encoder.h
   @author TEAM TEN
   @version 1.0
   @brief Helper class to abstract away communication with our encoders.
*/

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

class Encoder {
  public:
    Encoder(uint8_t address);
    uint16_t read();
  private:
    const uint8_t address_ = 0x40;
};

#endif
