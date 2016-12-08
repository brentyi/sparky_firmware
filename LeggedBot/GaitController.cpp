#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Wire.h>
#include "GaitController.h"
#include "Configuration.h"

GaitController::GaitController(const ConfigData* config) :
  config_(config),
  pwm_(config->pca9685_address),
  twist_linear_(0),
  twist_angular_(0),
  alternate_phase_(true)
{}

void GaitController::init() {
  Wire.begin();

  for (uint8_t x = 0; x < LEG_SIDES_X; x++) {
    for (uint8_t y = 0; y < LEG_SIDES_Y; y++) {
      leg_[x][y] = new LegController(config_, static_cast<LegSideX>(x), static_cast<LegSideY>(y));

      if (config_->hbridge_pin_a[x][y] < 100)
        pinMode(config_->hbridge_pin_a[x][y], OUTPUT);
      if (config_->hbridge_pin_b[x][y] < 100)
        pinMode(config_->hbridge_pin_b[x][y], OUTPUT);
    }
  }

  pwm_.begin();
  pwm_.setPWMFreq(1600);
  delay(500);

  // disable motors if battery voltage too low (ie powered by USB)
  pinMode(config_->batt_pin, INPUT);
  setPin_(config_->enable_pin, getBatteryVoltage() > config_->voltage_cutoff);

  pwm_.setPin(config_->led_channel, 4095);
  delay(500);
  pwm_.setPin(config_->led_channel, 0);
  delay(500);
  pwm_.setPin(config_->led_channel, 4095);

  next_step_time_ = millis();
}

/**
   @brief Non-violently stand robot up.
*/
void GaitController::stand() {
  uint32_t now = millis();
  for (uint8_t x = 0; x < LEG_SIDES_X; x++) {
    for (uint8_t y = 0; y < LEG_SIDES_Y; y++) {
      leg_[x][y]->setGoal(0, now + 3000, false, false);
    }
  }
}

/**
   @brief Update walk movements. This should be called as frequently as possible.
*/
void GaitController::update() {
  uint32_t now = millis();
  bool new_step = now >= next_step_time_;

  for (uint8_t x = 0; x < LEG_SIDES_X; x++) {
    for (uint8_t y = 0; y < LEG_SIDES_Y; y++) {

      if (new_step) {
        if (abs(twist_linear_) < 0.0001 && abs(twist_angular_) < 0.0001) {
          // no desired movement => all legs to "stand" pose
          leg_[x][y]->setGoal(0, now);
        } else {
          // TODO: actual math for calculating leg trajectories
          // this only does forward + back + left + right at fixed velocities

          // TODO: things to implement for trajectories meant for ground contact:
          //       - more aggressive feedback control? maybe an integral term?

          bool backwards = false;
          if (abs(twist_angular_) > 0.0001) {
            backwards = (x == 0) ^ (twist_angular_ > 0);
          } else {
            backwards = (twist_linear_ < 0);
          }

          if (((x + y) % 2 == 0) ^ alternate_phase_ ^ backwards) { // contacting ground
            leg_[x][y]->setGoal(config_->gait_contact_angle, now + config_->gait_step_duration, backwards, !backwards);
          } else { // going through air
            leg_[x][y]->setGoal(-config_->gait_contact_angle, now + config_->gait_step_duration, backwards, backwards);
          }
        }
        next_step_time_ = now + config_->gait_step_duration + 100;
      }

      int16_t effort = (int16_t) (4095.0 * leg_[x][y]->calculateEffort());
      if (config_->invert_motor[x][y]) {
        effort = -effort;
      }

      if (abs(effort) < 250) {
        // save power
        effort = 0;
      }

      setPin_(config_->hbridge_pin_a[x][y], effort > 0);
      setPin_(config_->hbridge_pin_b[x][y], effort < 0);
      pwm_.setPin(config_->pwm_channel[x][y], static_cast<uint16_t>(abs(effort)));
    }
  }

  alternate_phase_ ^= new_step;
}

/**
   @brief Abstraction for setting output states through our available hardware devices
   @param pin Pin number; >100 represents channels on our PWM mux
   @param value New pin state
*/
void GaitController::setPin_(uint8_t pin, bool value) {
  if (pin < 100)
    digitalWrite(pin, value);
  else
    pwm_.setPin(pin - 100, value ? 4095 : 0);
}

/**
   @brief Assign setpoints for our overall linear and angular velocities.
   @param linear Linear velocity setpoint (m/sec, x axis)
   @param angular Angular velocity setpoint (rad/sec, z axis)
*/
void GaitController::setTwist(float linear, float angular) {
  twist_linear_ = linear;
  twist_angular_ = angular;
}

/**
   @brief Read & return the current battery voltage.
   @return Battery voltage
*/
float GaitController::getBatteryVoltage() {
  return analogRead(config_->batt_pin) / 1023.0 * 5.0 * 3.10 / 1.0;
}

/**
   @brief Error detected -- stop all motors & blink LED
*/
void GaitController::fault() {
  setPin_(config_->enable_pin, LOW);
  for (;;) {
    pwm_.setPin(config_->led_channel, 4095);
    delay(1000);
    pwm_.setPin(config_->led_channel, 0);
    delay(1000);
  }
}

