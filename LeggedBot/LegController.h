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
  public:
    ControlMode mode_;
    ControlMode prev_mode_;

    PID* position_pid_;
    PID* velocity_pid_;

    // using doubles here because the pid library demands them :(
    // (same as floats)
    double position_;
    double velocity_;
    double position_setpoint_;
    double velocity_setpoint_;

    double control_effort_;

    float readPosition_();

    uint32_t prev_time_;

    uint16_t zero_;
    bool invert_encoder_;

    // TODO: our leg goal/trajectory can probably be described more concisely
    bool goal_arrived_;
    float goal_start_position_;
    float goal_end_position_;
    uint32_t goal_end_time_;
    uint32_t goal_start_time_;

    bool backwards_;

    inline float wrapAngle_(float theta);

  //public:
    AMS_AS5048B* encoder_;
    LegController(ConfigData* config, LegSideX x, LegSideY y);
    float calculateEffort();
    void setGoal(float destination, uint32_t arrival_time, bool backwards=false);
};

#endif /* GAITCONTROLLER_H */
