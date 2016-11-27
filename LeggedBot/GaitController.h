/**
 * @file GaitController.h
 * @author TEAM TEN
 * @version 1.0
 * @brief Class for generating and handling coordinated leg trajectories.
 */

#ifndef GAITCONTROLLER_H
#define GAITCONTROLLER_H

#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>
#include "Configuration.h"
#include "LegController.h"

class GaitController {
  private:
    ConfigData* config_;
    Adafruit_PWMServoDriver* pwm_;

    // forward velocity setpoint (chassis)
    float twist_linear_;
    // angular velocity setpoint (chassis)
    float twist_angular_;

    uint32_t next_step_;

    bool alternate_phase_;

    void setPin_(uint8_t pin, bool value);

    LegController* leg_[LEG_SIDES_X][LEG_SIDES_Y];
  public:
    GaitController(ConfigData* config);
    void init();
    void update();
    void setTwist(float linear, float angular);
};

#endif /* GAITCONTROLLER_H */
