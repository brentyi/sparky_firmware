#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

class Encoder {
  public:
    uint16_t read();
    uint8_t address = 0x40;
};

#endif
