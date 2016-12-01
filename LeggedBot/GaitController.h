/**
   @file GaitController.h
   @author TEAM TEN
   @version 1.0
   @brief Interface for controlling our robot!
*/

#ifndef GAITCONTROLLER_H
#define GAITCONTROLLER_H

#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>
#include "Configuration.h"
#include "LegController.h"

class GaitController {
  public:
    GaitController(ConfigData* config);
    void update();
    void setTwist(float linear, float angular);
    float getBatteryVoltage();
    void fault();

  private:
    ConfigData* config_;
    Adafruit_PWMServoDriver* pwm_;

    // forward velocity setpoint (chassis)
    float twist_linear_;
    // angular velocity setpoint (chassis)
    float twist_angular_;

    bool alternate_phase_;

    uint32_t next_step_time_;

    void setPin_(uint8_t pin, bool value);

    LegController* leg_[LEG_SIDES_X][LEG_SIDES_Y];
};

#endif /* GAITCONTROLLER_H */
