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

  mode_ = PID_VELOCITY;
  position_ = readPosition_();
  velocity_ = 0;
  prev_time_ = millis();
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
  float diff; // generic angular difference
  
  // Update encoder data
  float new_position = readPosition_();
  uint32_t new_time = millis();
  diff = new_position - position_;
  if (diff > PI) {
    diff -= TWO_PI;
  } else if (diff < -PI) {
    diff += TWO_PI;
  }
  // calculate angular velocity -- radians/sec
  velocity_ = diff / (new_time - prev_time_) * 1000.0;
  prev_time_ = new_time;


  diff = position_setpoint_ - new_position;
  if (diff > PI) {
    new_position += TWO_PI;
  } else if (diff < -PI) {
    new_position -= TWO_PI;
  }
  position_ = new_position;

  // Update control loops
  switch (mode_) {
    case LEG_OFF:
      velocity_pid_->SetMode(MANUAL);
      position_pid_->SetMode(MANUAL);
      control_effort_ = 0;
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
  position_pid_->Compute();

  return control_effort_;
}

float LegController::readPosition_() {
  int16_t raw_angle = (int16_t)encoder_->angleRegR() - zero_;
  float rad = raw_angle / 8192.0 * PI;
  if (invert_encoder_) {
    rad = -rad;
  }
  return fmod(fmod(rad + PI, TWO_PI) + TWO_PI, TWO_PI) - PI;
}

/**
   @brief Set a desired leg position, at a given time.
   @param destination Goal position, in radians.
   @param arrival_time Expected arrival time for our goal position, in milliseconds.
   @param backwards Use backwards leg movement.
*/
void LegController::setGoal(float destination, uint32_t arrival_time, bool backwards) {
  return;
}

