/**
   @file Configuration.h
   @author TEAM TEN
   @version 1.0
   @brief (Experimental) Configuration class for dynamically configurable robot settings, which can be pulled from and pushed to the EEPROM.

   Storing all of our configuration data in a struct obviously isn't as efficient as doing so in preprocessor directives, but I was using a bunch
   of arrays anyways and it'd be super neat if we could change all this low-level stuff on the fly: control loop tuning is the most obvious
   application, but this'd also enable self-calibration, encoder realignment, PWM channel swapping, etc.
*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
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
  uint8_t as5048b_address[LEG_SIDES_X][LEG_SIDES_Y] {{0b1000000, 0b1000110, 0b1000101}, {0b1000100, 0b1000111, 0b1000010}};
  uint8_t pca9685_address{0b1100000};

  // Invert values?
  bool invert_encoder[LEG_SIDES_X][LEG_SIDES_Y] {{false, false, false}, {false, false, false}};
  bool invert_motor[LEG_SIDES_X][LEG_SIDES_Y] {{true, false, false}, {true, true, false}};

  //
  // Pin numbers >100 represent channels on our PWM mux (-100)
  //

  // PWM channel numbering
  uint8_t pwm_channel[LEG_SIDES_X][LEG_SIDES_Y] {{14, 12, 10}, {13, 11, 9}};

  // Voltage reading
  uint8_t batt_pin{A0};

  // Motor driver enable
  uint8_t enable_pin{103};

  // H-bridge direction pins
  int8_t hbridge_pin_a[LEG_SIDES_X][LEG_SIDES_Y] {{A3, 5, 6}, {8, 104, 3}};
  int8_t hbridge_pin_b[LEG_SIDES_X][LEG_SIDES_Y] {{A2, 4, 7}, {A1, 105, 2}};
  
} ConfigData;

class Configuration {
  public:
    Configuration();
    ConfigData data;
    void pull();
    void push();
};

#endif /* CONFIGURATION_H */
