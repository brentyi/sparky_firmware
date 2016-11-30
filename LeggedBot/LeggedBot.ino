#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_PWMServoDriver.h>

#include "Configuration.h"
#include "BluefruitConfig.h"
#include "GaitController.h"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Configuration config;
GaitController* gc;

char voltage[6];
bool report_voltage;

void setup() {
  Serial.begin(115200);

  delay(500);

  gc = new GaitController(&config.data);

  if (gc->getBatteryVoltage() <= config.data.voltage_cutoff) {
    gc->fault();
  }

  gc->setTwist(0, 0);

  ble.begin(VERBOSE_MODE);
  ble.echo(false);
  ble.info();
  ble.verbose(false);

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }
  ble.setMode(BLUEFRUIT_MODE_DATA);
  ble.setBleUartRxCallback(callback);
}

void loop() {
  ble.update();
  gc->update();

  if (report_voltage) {    
    dtostrf(gc->getBatteryVoltage(), 4, 4, voltage);
    ble.print(F("Battery voltage: "));
    ble.println(voltage);
    report_voltage = false;x
  }
}

void callback(char data[], uint16_t len) {
  if (len == 0) return;

  /* Got a packet! */
  if (data[1] == 'B') { //buttons
    uint8_t buttnum = data[2] - '0';
    boolean pressed = data[3] - '0';
    Serial.print ("Button "); Serial.println(buttnum);
    if (buttnum == 5 && pressed) {
      gc->setTwist(1, 0);
    } else if (buttnum == 6 && pressed) {
      gc->setTwist(-1, 0);
    } else if (buttnum == 7 && pressed) {
      gc->setTwist(0, -1);
    } else if (buttnum == 8 && pressed) {
      gc->setTwist(0, 1);
    } else if (buttnum >= 5 && buttnum <= 8 && !pressed) {
      gc->setTwist(0, 0);
    } else if (buttnum == 1 && pressed) {
      config.data.gait_contact_angle = 0.2;
      config.data.gait_step_duration = 800;
    } else if (buttnum == 2 && pressed) {
      config.data.gait_contact_angle = 0.3;
      config.data.gait_step_duration = 800;
    } else if (buttnum == 3 && pressed) {
      config.data.gait_contact_angle = 0.4;
      config.data.gait_step_duration = 800;
    } else if (buttnum == 4 && pressed) {
      config.data.gait_contact_angle = 0.5;
      config.data.gait_step_duration = 800;
    }

    if (gc->getBatteryVoltage() <= config.data.voltage_cutoff) {
      gc->fault();
    }
  } else if (strcmp(data, "batt") == 0) {
    report_voltage = true;
  }
}

