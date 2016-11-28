/**
   @file LegController.h
   @author TEAM TEN
   @version 1.0
   @brief Position and velocity controller for individual robot legs.
*/

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include <ams_as5048b.h>
#include <stdint.h>
#include <PID_v1.h>
#include "Configuration.h"

typedef enum ControlMode {
  LEG_OFF,
  PID_POSITION
} ControlMode;

class LegController {
  public:
    AMS_AS5048B* encoder_;
    LegController(ConfigData* config, LegSideX x, LegSideY y);
    float calculateEffort();
    void setGoal(float destination, uint32_t arrival_time, bool backwards = false);
  private:
    ControlMode mode_;
    ControlMode prev_mode_;

    PID* position_pid_;

    // using doubles here because the pid library demands them -- functionally same as float
    double position_;
    double position_pid_input_; // position control input -- same as position_ but +/-2pi for angle wrapping reasons
    double position_setpoint_;

    double control_effort_;

    float readPosition_();

    uint16_t zero_;
    bool invert_encoder_;

    bool travel_arrived_;
    float travel_start_position_;
    float travel_end_position_;
    uint32_t travel_start_time_;
    uint32_t travel_end_time_;

    bool backwards_;

    float wrapAngle_(float theta);
};

#endif /* GAITCONTROLLER_H */
