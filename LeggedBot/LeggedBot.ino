#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_PWMServoDriver.h>

#include "Configuration.h"
#include "BluefruitConfig.h"
#include "GaitController.h"


Configuration config;
GaitController robot(&config.data);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

bool report_status;
uint32_t prev_time;

void setup() {
  Serial.begin(115200);

  delay(500);

  Serial.println("--------------------");

  robot.init();
  robot.setTwist(0, 0);

  ble.begin(VERBOSE_MODE);
  if ( ! ble.factoryReset() ) {
    Serial.println(F("Couldn't factory reset"));
    robot.fault();
  }
  ble.echo(false);
  ble.info();

  float voltage = robot.getBatteryVoltage();

  char rename_cmd[40] = "AT+GAPDEVNAME=TEAM TEN ";
  dtostrf(voltage, 4, 4, rename_cmd + strlen(rename_cmd)); // ...is this an acceptable way to concat a float?
  ble.sendCommandCheckOK(rename_cmd);
  ble.verbose(false);

  if (voltage <= config.data.voltage_cutoff && !(voltage > 4 && voltage < 4.5)) {
    robot.fault();
  }

  Serial.print(F("Waiting for BLE connection.... "));
  while (!ble.isConnected()) {
    delay(500);
  }
  Serial.println(F("connected!"));

  ble.setMode(BLUEFRUIT_MODE_DATA);
  ble.setBleUartRxCallback(callback);

  prev_time = millis();
}

void loop() {
  ble.update();
  robot.update();

  uint32_t now = millis();
  if (report_status) {
    char temp[7];

    ble.println("--------------");
    ble.print(F("Main loop frequency: \t"));
    dtostrf( 1000.0 / (now - prev_time), 4, 4, temp);
    ble.print(temp);
    ble.println("Hz");

    dtostrf(robot.getBatteryVoltage(), 4, 4, temp);
    ble.print(F("Battery voltage: \t"));
    ble.println(temp);
    report_status = false;
  }
  prev_time = now;
}

void callback(char data[], uint16_t len) {
  if (len == 0) return;

  /* Got a packet! */
  if (data[1] == 'B') { //buttons
    uint8_t buttnum = data[2] - '0';
    boolean pressed = data[3] - '0';
    Serial.print ("Button "); Serial.println(buttnum);
    if (buttnum == 5 && pressed) {
      robot.setTwist(1, 0);
    } else if (buttnum == 6 && pressed) {
      robot.setTwist(-1, 0);
    } else if (buttnum == 7 && pressed) {
      robot.setTwist(0, -1);
    } else if (buttnum == 8 && pressed) {
      robot.setTwist(0, 1);
    } else if (buttnum >= 5 && buttnum <= 8 && !pressed) {
      robot.setTwist(0, 0);
    } else if (buttnum == 1 && pressed) {
      config.data.gait_contact_angle = 0.2;
      config.data.gait_step_duration = 800;
    } else if (buttnum == 2 && pressed) {
      config.data.gait_contact_angle = 0.3;
      config.data.gait_step_duration = 800;
    } else if (buttnum == 3 && pressed) {
      config.data.gait_contact_angle = 0.4;
      config.data.gait_step_duration = 600;
    } else if (buttnum == 4 && pressed) {
      config.data.gait_contact_angle = 0.7;
      config.data.gait_step_duration = 400;
    }

    if (robot.getBatteryVoltage() <= config.data.voltage_cutoff) {
      robot.fault();
    }
  } else if (strcmp(data, "status") == 0) {
    report_status = true;
  }
}

