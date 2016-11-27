#include <stdint.h>
#include <PID_v1.h>
#include <ams_as5048b.h>
#include "Configuration.h"
#include "LegController.h"

LegController::LegController(ConfigData* config, LegSideX x, LegSideY y) {
  encoder_ = new AMS_AS5048B(config->as5048b_address[x][y]);
  encoder_->begin();

  velocity_pid_ = new PID(&velocity_,
                          &control_effort_,
                          &velocity_setpoint_,
                          config->velocity_kp,
                          config->velocity_ki,
                          config->velocity_kd,
                          DIRECT);
  position_pid_ = new PID(&position_,
                          &control_effort_,
                          &position_setpoint_,
                          config->position_kp,
                          config->position_ki,
                          config->position_kd,
                          DIRECT);

  velocity_pid_->SetOutputLimits(-1.0, 1.0);
  position_pid_->SetOutputLimits(-1.0, 1.0);

  zero_ = config->leg_zero[x][y];
  invert_encoder_ = config->invert_encoder[x][y];

  mode_ = PID_POSITION;
  position_ = readPosition_();
  velocity_ = 0;
  prev_time_ = millis();
  goal_end_time_ = millis();
  goal_start_time_ = millis();
  goal_arrived_ = false;
  position_setpoint_ = 0;
  velocity_setpoint_ = TWO_PI;
  control_effort_ = 0;
  return;
}

/**
   @brief Update leg controls. This should be called as frequently as possible.
   @returns Control effort, -1.0 to 1.0
*/
float LegController::calculateEffort() {
  float diff; // generic/temporary variable for angular difference

  // Update encoder data
  float new_position = readPosition_();
  uint32_t now = millis();
  diff = new_position - position_;
  if (diff > PI) {
    diff -= TWO_PI;
  } else if (diff < -PI) {
    diff += TWO_PI;
  }
  // calculate angular velocity -- radians/sec
  velocity_ = diff / (now - prev_time_) * 1000.0;
  prev_time_ = now;

  position_ = new_position;

  // Update control mode based on goal
  diff = goal_end_position_ - position_;
  if (goal_arrived_ || goal_end_time_ <= now || abs(diff) < 0.001) {
    mode_ = PID_POSITION;
    position_setpoint_ = goal_end_position_;
    goal_arrived_ = true;
  } else {
    // TODO: this bit can be much better optimized + cleaned up
    diff = wrapAngle_(goal_end_position_ - goal_start_position_);
    if (backwards_ && diff > 0) {
      diff -= TWO_PI;
    } else if (!backwards_ && diff < 0) {
      diff += TWO_PI;
    }
    mode_ = PID_POSITION;
    diff *= now - goal_start_time_;
    diff /= goal_end_time_ - goal_start_time_;
    position_setpoint_ = goal_start_position_ + diff;
    position_setpoint_ = wrapAngle_(position_setpoint_);
    /*
    mode_ = PID_VELOCITY;
    velocity_setpoint_ = diff / (goal_time_ - now) * 1000.0;
    */
  }

  // Update control loops
  control_effort_ = 0;
  switch (mode_) {
    case LEG_OFF:
      velocity_pid_->SetMode(MANUAL);
      position_pid_->SetMode(MANUAL);
      break;
    case PID_VELOCITY:
      velocity_pid_->SetMode(AUTOMATIC);
      position_pid_->SetMode(MANUAL);
      break;
    case PID_POSITION:
      velocity_pid_->SetMode(MANUAL);
      position_pid_->SetMode(AUTOMATIC);
      break;
  }

  velocity_pid_->Compute();

  // Angular wraparound for position control
  /*diff = position_setpoint_ - position_;
    if (diff > PI) {
    position_ += TWO_PI;
    } else if (diff < -PI) {
    position_ -= TWO_PI;
    }*/
  position_pid_->Compute();
  // Undo any changes made by the wraparound fix
  position_ = new_position;

  return control_effort_;
}

float LegController::readPosition_() {
  int16_t raw_angle = (int16_t)encoder_->angleRegR() - zero_;
  float rad = raw_angle / 8192.0 * PI;
  if (invert_encoder_) {
    rad = -rad;
  }
  return wrapAngle_(rad);
}

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
  goal_end_position_ = destination;
  goal_start_position_ = position_;
  goal_start_time_ = millis(); // we may want to pass this in as an argument
  goal_end_time_ = arrival_time;
  goal_arrived_ = false;
  backwards_ = backwards;
  position_setpoint_ = destination;
  return;
}

