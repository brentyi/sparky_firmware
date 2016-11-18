/**
   @file Configuration.h
   @author TEAM TEN
   @version 1.0
   @brief Configuration class for dynamically configurable robot settings, which can be pulled from and pushed to the EEPROM.
*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdint.h>

// Standard units: millimeters, radians, seconds

enum LegSideX {
  LEFT = 0,
  RIGHT,
  LEG_SIDES_X
};

enum LegSideY {
  FRONT = 0,
  MIDDLE,
  BACK,
  LEG_SIDES_Y
};

typedef struct ConfigData {
  // Walking style
  float gait_contact_angle{0.1};
  float gait_angular_velocity{0.1};
  float leg_radius{0};

  // Controls
  float position_kp{10};
  float position_ki{0};
  float position_kd{0};

  float velocity_kp{10};
  float velocity_ki{0};
  float velocity_kd{0};

  // Encoder offsets for each leg
  uint16_t leg_zero[LEG_SIDES_X][LEG_SIDES_Y] {{0, 0, 0 }, {0, 0, 0}};

  // Shared angular offset (radians)
  float leg_offset{0};

  // I2C addresses
  uint8_t as5048b_address[LEG_SIDES_X][LEG_SIDES_Y] {{0, 0, 0 }, {0, 0, 0}};

  // Invert values?
  bool invert_encoder[LEG_SIDES_X][LEG_SIDES_Y] {{false, false, false}, {false, false, false}};
  bool invert_motor[LEG_SIDES_X][LEG_SIDES_Y] {{false, false, false}, {false, false, false}};

  // PWM channel numbering
  uint8_t pwm_channel[LEG_SIDES_X][LEG_SIDES_Y] {{0, 0, 0 }, {0, 0, 0}};

  // H-bridge pins
  uint8_t hbridge_pin_a[LEG_SIDES_X][LEG_SIDES_Y] {{0, 0, 0 }, {0, 0, 0}};
  uint8_t hbridge_pin_b[LEG_SIDES_X][LEG_SIDES_Y] {{0, 0, 0 }, {0, 0, 0}};
} ConfigData;

class Configuration {
  public:
    Configuration();
    ConfigData data;
    void pull();
    void push();
};

#endif /* CONFIGURATION_H */
