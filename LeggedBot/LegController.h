/**
   @file LegController.h
   @author TEAM TEN
   @version 1.0
   @brief Position and velocity controller for individual robot legs.
*/

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include <stdint.h>
#include <PID_v1.h>
#include "Configuration.h"
#include "Encoder.h"

typedef enum ControlMode {
  LEG_OFF,
  PID_POSITION
} ControlMode;

class LegController {
  public:
    Encoder encoder_;
    LegController(const ConfigData* config, LegSideX x, LegSideY y);
    float calculateEffort();
    void setGoal(float destination, uint32_t arrival_time, bool backwards = false, bool shortest_path = true);

  private:
    const ConfigData* config_;

    ControlMode mode_;

    PID* position_pid_;

    // using doubles here because the pid library demands them -- functionally same as float
    double position_;
    double position_pid_input_; // position control input -- same as position_ but +/-2pi for angle wrapping reasons
    double position_setpoint_;

    float velocity_;

    float feedforward_effort_;
    double feedback_effort_;

    void readState_();

    uint32_t prev_time_;
    const uint16_t zero_;
    const bool invert_encoder_;

    float travel_start_position_;
    float travel_total_offset_;
    uint32_t travel_start_time_;
    uint32_t travel_end_time_;

    static float wrapAngle_(float theta);
    float wrapAngleDirected_(float theta);
};

#endif /* GAITCONTROLLER_H */
