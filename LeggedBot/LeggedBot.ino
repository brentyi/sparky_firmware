#include "Configuration.h"
#include "GaitController.h"
#include <Adafruit_PWMServoDriver.h>

Configuration config;
GaitController* gc;

void setup() {
  Serial.begin(9600);
  
  gc = new GaitController(&config.data);
  gc->setTwist(1, 0);
}

void loop() {
  gc->update();
}
