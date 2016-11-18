/**
 * @file GaitController.h
 * @author TEAM TEN
 * @version 1.0
 * @brief Class for generating and handling coordinated leg trajectories.
 */

#ifndef GAITCONTROLLER_H
#define GAITCONTROLLER_H

#include <stdint.h>
#include "Configuration.h"
#include "LegController.h"

class GaitController {
  private:
    ConfigData* config_;

    // forward velocity setpoint (chassis)
    float twist_linear_;
    // angular velocity setpoint (chassis)
    float twist_angular_;

    LegController* leg_[LEG_SIDES_X][LEG_SIDES_Y];
  public:
    GaitController(ConfigData* config): config_(config);
    void update(uint32_t time);
    void setTwist(float linear, float angular);
};

#endif /* GAITCONTROLLER_H */
