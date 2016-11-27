#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "GaitController.h"
#include "Configuration.h"

GaitController::GaitController(ConfigData* config) : config_(config) {
  twist_linear_ = 0;
  twist_angular_ = 0;
  alternate_phase_ = true;
}

/**
   @brief Hardware initialization steps for our gait controller.
*/
void GaitController::init() {
  Serial.print(".");
  for (uint8_t x = 0; x < LEG_SIDES_X; x++) {
    for (uint8_t y = 0; y < LEG_SIDES_Y; y++) {
      leg_[x][y] = new LegController(config_, static_cast<LegSideX>(x), static_cast<LegSideY>(y));

      if (config_->hbridge_pin_a[x][y] < 100)
        pinMode(config_->hbridge_pin_a[x][y], OUTPUT);
      if (config_->hbridge_pin_b[x][y] < 100)
        pinMode(config_->hbridge_pin_b[x][y], OUTPUT);
    }
  }
  Serial.print(".");
  pwm_ = new Adafruit_PWMServoDriver(config_->pca9685_address);
  Serial.print(".");
  pwm_->begin();
  Serial.print(".");
  pwm_->setPWMFreq(1600);
  Serial.print(".");

  // disable motors if battery voltage too low (ie powered by USB)
  pinMode(config_->batt_pin, INPUT);
  delay(100);
  uint16_t level = analogRead(config_->batt_pin);
  bool power = level > 450;
  pwm_->setPin(config_->led_channel, 0);
  setPin_(config_->enable_pin, power);
  Serial.print("Battery at: ");
  Serial.println(level);
  if (power) {
    /*for (uint16_t i = 0; i < 4095; i += 3) {
      pwm_->setPin(config_->led_channel, i);
      delay(1);
      }
      delay(500);
      pwm_->setPin(config_->led_channel, 0);
      delay(500);*/
    pwm_->setPin(config_->led_channel, 4095);
    delay(500);
    pwm_->setPin(config_->led_channel, 0);
    delay(500);
    pwm_->setPin(config_->led_channel, 4095);
  }

  next_step_ = millis();
}

/**
   @brief Update walk movements. This should be called as frequently as possible.
*/
void GaitController::update() {
  uint32_t now = millis();
  bool new_step = now >= next_step_;

  for (uint8_t x = 0; x < LEG_SIDES_X; x++) {
    for (uint8_t y = 0; y < LEG_SIDES_Y; y++) {
      // control mode definition
      if (new_step) {
        if (abs(twist_linear_) < 0.0001 && abs(twist_angular_) < 0.0001) {
          // no desired movement => all legs to "stand" pose
          leg_[x][y]->setGoal(0, now);
        } else if (twist_linear_ > 0.0001) {
          // walk forward!
          // TODO: math for actual stride length calculation + omega stuff
          if (((x + y) % 2 == 0) ^ alternate_phase_) {
            // ground contact
            leg_[x][y]->setGoal(-config_->gait_contact_angle, now + config_->gait_step_duration);
          } else {
            // through the air!
            leg_[x][y]->setGoal(config_->gait_contact_angle, now + config_->gait_step_duration);
          }
        }
        next_step_ = now + config_->gait_step_duration + 100;
      }

      // run control loops
      int16_t effort = (int16_t) (4095 * leg_[x][y]->calculateEffort());
      if (config_->invert_motor[x][y]) {
        effort = -effort;
      }

      if (abs(effort) < 150) {
        // powersave
        effort = 0;
      }

      Serial.print(leg_[x][y]->position_);
      Serial.print(",");
      Serial.print(leg_[x][y]->position_setpoint_);
      Serial.print("\t");

      //effort = 1000;

      setPin_(config_->hbridge_pin_a[x][y], effort > 0);
      setPin_(config_->hbridge_pin_b[x][y], effort < 0);
      pwm_->setPin(config_->pwm_channel[x][y], (uint16_t) abs(effort));
    }
  }

  alternate_phase_ ^= new_step;
  
  Serial.println();
  return;
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
    pwm_->setPin(pin - 100, value ? 4095 : 0);
}

/**
   @brief Assign setpoints for our overall linear and angular velocities.
   @param linear Linear velocity setpoint (m/sec, x axis)
   @param angular Angular velocity setpoint (rad/sec, z axis)
*/
void GaitController::setTwist(float linear, float angular) {
  twist_linear_ = linear;
  twist_angular_ = angular;
  return;
}

