#include <stdint.h>
#include <PID_v1.h>
#include "Encoder.h"
#include "Configuration.h"
#include "LegController.h"

LegController::LegController(const ConfigData* config, LegSideX x, LegSideY y) :
  config_(config),
  encoder_(config->as5048b_address[x][y]),
  zero_(config->leg_zero[x][y]),
  invert_encoder_ (config->invert_encoder[x][y]),
  travel_arrived_(true),
  travel_start_position_(0),
  travel_end_position_(0),
  position_setpoint_(0),
  feedback_effort_(0),
  mode_(PID_POSITION)
{

  // Populate leg state variables
  readState_();
  velocity_ = 0;

  // Travel command setup
  travel_start_time_ = millis();
  travel_end_time_ = millis();

  // Feedback control setup
  position_pid_ = new PID(&position_pid_input_,
                          &feedback_effort_,
                          &position_setpoint_,
                          config->position_kp,
                          config->position_ki,
                          config->position_kd,
                          DIRECT);
  position_pid_->SetOutputLimits(-1.0, 1.0);
}

/**
   @brief Update leg controls. This should be called as frequently as possible.
   @returns Control effort, -1.0 to 1.0
*/
float LegController::calculateEffort() {
  uint32_t now = millis();

  float diff; // generic angular difference -- TODO: more verbose/individual naming?

  float feedforward_effort = 0;

  readState_();

  // Update setpoints based on goal
  diff = wrapAngle_(travel_end_position_ - position_);
  if (travel_arrived_ || travel_end_time_ <= now || abs(diff) < 0.01) {
    mode_ = PID_POSITION;
    position_setpoint_ = travel_end_position_;
    travel_arrived_ = true;
  } else {
    // TODO: there's quite a bit of redundant floating point math that could be avoided by precomputing
    if (backwards_ && diff > 0) {
      diff -= TWO_PI;
    } else if (!backwards_ && diff < 0) {
      diff += TWO_PI;
    }
    feedforward_effort = wrapAngleDirected_(diff) * 1000.0 / (travel_end_time_ - now) * config_->velocity_ff;
    diff = wrapAngleDirected_(travel_end_position_ - travel_start_position_);
    diff *= now - travel_start_time_;
    diff /= travel_end_time_ - travel_start_time_;
    position_setpoint_ = wrapAngle_(travel_start_position_ + diff);

    mode_ = PID_POSITION;
  }

  // Wrap angle for position control
  position_pid_input_ = position_;
  diff = position_setpoint_ - position_pid_input_;
  if (diff > PI) {
    position_pid_input_ += TWO_PI;
  } else if (diff < -PI) {
    position_pid_input_ -= TWO_PI;
  }

  // Update control loops
  switch (mode_) {
    case LEG_OFF:
      position_pid_->SetMode(MANUAL);
      feedback_effort_ = 0;
      break;
    case PID_POSITION:
      position_pid_->SetMode(AUTOMATIC);
      break;
  }

  position_pid_->Compute();

  return constrain(feedback_effort_ + feedforward_effort, -1.0, 1.0);
}

/**
   @brief Read and return our current leg position.
*/
void LegController::readState_() {
  uint32_t now = millis();

  int16_t raw_angle = (int16_t)encoder_.read() - zero_;
  float new_position = wrapAngle_(raw_angle / 8192.0 * PI);
  if (invert_encoder_) {
    new_position = -new_position;
  }
  velocity_ = wrapAngle_(new_position - position_) / (now - prev_time_) * 1000.0;
  position_ = new_position;

  prev_time_ = now;
}

/**
   @brief Helper for normalizing angles.
   @param theta Angle to normalize
*/
float LegController::wrapAngle_(float theta) {
  return fmod(fmod(theta + PI, TWO_PI) + TWO_PI, TWO_PI) - PI;
}

/**
   @brief Helper for normalizing angles relative to our travel direction.
   @param theta Angle to normalize
*/
float LegController::wrapAngleDirected_(float theta) {
  theta = wrapAngle_(theta);
  if (backwards_ && theta > 0) {
    return theta - TWO_PI;
  } else if (!backwards_ && theta < 0) {
    return theta + TWO_PI;
  }
  return theta;
}

/**
   @brief Set a desired leg position, at a given time.
   @param destination Goal position, in radians.
   @param arrival_time Expected arrival time for our goal position, in milliseconds.
   @param backwards Use backwards leg movement.
*/
void LegController::setGoal(float destination, uint32_t arrival_time, bool backwards) {
  travel_start_position_ = position_;
  travel_end_position_ = destination;
  travel_start_time_ = millis();
  travel_end_time_ = arrival_time;
  backwards_ = backwards;
  travel_arrived_ = false;
}
