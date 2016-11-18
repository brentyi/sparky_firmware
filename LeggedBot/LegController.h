/**
 * @file LegController.h
 * @author TEAM TEN
 * @version 1.0
 * @brief Position and velocity controller for individual robot legs.
 */

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include <ams_as5048b.h>
#include <stdint.h>
#include <PID_v1.h>
#include "Configuration.h"

typedef enum ControlMode {
  LEG_OFF,
  PID_VELOCITY,
  PID_POSITION
} ControlMode;

class LegController {
  private:
    ControlMode mode_;
    ControlMode prev_mode_;

    PID position_pid_;
    PID velocity_pid_;

    float position_setpoint_;
    float velocity_setpoint_;

    AMS_AS5048B encoder_;

    float zero_;

    uint16_t angle_raw_;
    float angle_radians_;
    float goal_angle_;
    uint32_t goal_time_;

    bool backwards_;

  public:
    LegController(ConfigData* config, LegSideX x, LegSideY y);
    float calculateEffort();
    void setGoal(float destination, uint32_t arrival_time, bool backwards);
};

#endif /* GAITCONTROLLER_H */
