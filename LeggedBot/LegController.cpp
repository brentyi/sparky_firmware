#include <stdint.h>
#include <PID_v1.h>
#include <ams_as5048b.h>
#include "Configuration.h"
#include "LegController.h"

LegController::LegController(ConfigData* config, LegSideX x, LegSideY y) {
  encoder_ = new AMS_AS5048B(config->as5048b_address[x][y]);
  encoder_->begin();

  zero_ = config->leg_zero[x][y];
  invert_encoder_ = config->invert_encoder[x][y];

  mode_ = PID_POSITION;
  position_ = readPosition_();
  position_setpoint_ = 0;
  control_effort_ = 0;

  travel_arrived_ = true;
  travel_start_position_ = position_;
  travel_end_position_ = 0;
  travel_start_time_ = millis();
  travel_end_time_ = millis();

  position_pid_ = new PID(&position_pid_input_,
                          &control_effort_,
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

  float diff; // generic angular difference

  // Update encoder data
  position_ = readPosition_();

  // Update setpoints based on goal
  diff = wrapAngle_(travel_end_position_ - position_);
  if (travel_arrived_ || travel_end_time_ <= now || abs(diff) < 0.01) {
    mode_ = PID_POSITION;
    position_setpoint_ = travel_end_position_;
    travel_arrived_ = true;
  } else {
    // TODO: this bit can be much better optimized + cleaned up
    diff = wrapAngle_(travel_end_position_ - travel_start_position_);
    if (backwards_ && diff > 0) {
      diff -= TWO_PI;
    } else if (!backwards_ && diff < 0) {
      diff += TWO_PI;
    }
    mode_ = PID_POSITION;
    diff *= now - travel_start_time_;
    diff /= travel_end_time_ - travel_start_time_;
    position_setpoint_ = travel_start_position_ + diff;
    position_setpoint_ = wrapAngle_(position_setpoint_);
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
      control_effort_ = 0;
      break;
    case PID_POSITION:
      position_pid_->SetMode(AUTOMATIC);
      break;
  }

  position_pid_->Compute();

  return control_effort_;
}

/**
   @brief Read and return our current leg position.
*/
float LegController::readPosition_() {
  int16_t raw_angle = (int16_t)encoder_->angleRegR() - zero_;
  float rad = raw_angle / 8192.0 * PI;
  if (invert_encoder_) {
    rad = -rad;
  }
  return wrapAngle_(rad);
}

/**
   @brief Helper for normalizing angles.
   @param theta Angle to normalize
*/
float LegController::wrapAngle_(float theta) {
  return fmod(fmod(theta + PI, TWO_PI) + TWO_PI, TWO_PI) - PI;
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
