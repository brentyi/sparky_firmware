#include "Configuration.h"
#include "GaitController.h"
#include <Adafruit_PWMServoDriver.h>
#include <stdint.h>

Configuration config;
GaitController gc(&config.data);

void setup() {
  Serial.begin(9600);
  Serial.println("Starting");
  delay(100);
  gc.init();
  delay(100);
  for (uint16_t i = 0; i < 10000; i++) {
    gc.update();
  }
  //gc.setTwist(1, 0);
}

void loop() {
  gc.update();
  //delay(50); // TODO: get rid of this -- this is a hack to alleviate velocity measurement/noise issues with the front left encoder
}
