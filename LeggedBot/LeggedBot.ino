#include "Configuration.h"
#include "GaitController.h"
#include <Adafruit_PWMServoDriver.h>

Configuration config;
GaitController* gc;

void setup() {
  Serial.begin(9600);
  Serial.println("begin setup");
  gc = new GaitController(&config.data);
  Serial.println("end setup");
}

void loop() {
  /*
  digitalWrite(config.data.hbridge_pin_a[RIGHT][FRONT], LOW);
  digitalWrite(config.data.hbridge_pin_b[RIGHT][FRONT], HIGH);
  pwm.setPin(3, 4095);
  pwm.setPin(config.data.pwm_channel[RIGHT][FRONT], 4095);
  pwm.setPWM(8, 0, 4000);
  */
  //Serial.println("...");
  gc->update();
  //Serial.println(config.data.hbridge_pin_a[RIGHT][FRONT]);
}
