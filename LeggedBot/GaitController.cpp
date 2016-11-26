#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>
#include "GaitController.h"
#include "Configuration.h"

GaitController::GaitController(ConfigData* config) : config_(config) {
  
  for (uint8_t x = 0; x < LEG_SIDES_X; x++) {
    for (uint8_t y = 0; y < LEG_SIDES_Y; y++) {
      leg_[x][y] = new LegController(config_, static_cast<LegSideX>(x), static_cast<LegSideY>(y));

      if (config_->hbridge_pin_a[x][y] < 100)
        pinMode(config_->hbridge_pin_a[x][y], OUTPUT);
      if (config_->hbridge_pin_b[x][y] < 100)
        pinMode(config_->hbridge_pin_b[x][y], OUTPUT);
    }
  }
  Serial.println("begiadsan"); 
  pwm_ = new Adafruit_PWMServoDriver(config->pca9685_address);
  Serial.println("begino"); 
  pwm_->begin();
  pwm_->setPWMFreq(1600);  // This is the maximum PWM frequency

  // disable motors if battery voltage too low (ie powered by USB)
  pinMode(config_->batt_pin, INPUT);
  setPin_(config_->enable_pin, (analogRead(config_->batt_pin) > 450));
}

/**
   @brief Update walk movements. This should be called as frequently as possible.
*/
void GaitController::update() {
  for (uint8_t x = 0; x < LEG_SIDES_X; x++) {
    for (uint8_t y = 0; y < LEG_SIDES_Y; y++) {
      int16_t effort = (int16_t) (4095 * leg_[x][y]->calculateEffort());
      if (config_->invert_motor[x][y]) {
        effort = -effort;
      }
      
      setPin_(config_->hbridge_pin_a[x][y], effort > 0);
      setPin_(config_->hbridge_pin_b[x][y], effort < 0);
      pwm_->setPin(config_->pwm_channel[x][y], (uint16_t) abs(effort));
    }
  }
  return;
}

void GaitController::setPin_(uint8_t pin, bool value) {
  if (pin < 100) {
    digitalWrite(pin, value);
  } else {
    pwm_->setPin(pin - 100, value ? 4095 : 0);
  }
}

/**
   @brief Assign setpoints for our overall linear and angular velocities.
   @param linear Linear velocity setpoint (m/sec, x axis)
   @param angular Angular velocity setpoint (rad/sec, z axis)
*/
void GaitController::setTwist(float linear, float angular) {
  return;
}

