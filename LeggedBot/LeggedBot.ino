#include "Configuration.h"
#include "GaitController.h"
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
/*=========================================================================
    APPLICATION SETTINGS
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Configuration config;
GaitController* gc;

void setup() {
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);

  ble.begin(VERBOSE_MODE);
  ble.echo(false);
  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!

  gc = new GaitController(&config.data);
  gc->setTwist(0, 0);

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
  ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  ble.setBleUartRxCallback(callback);
}

void loop() {
  ble.update();
  gc->update();
}

void callback(char data[], uint16_t len) {
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);
  // Buttons
  if (data[1] == 'B') {
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
    }else if (buttnum == 3 && pressed) {
      config.data.gait_contact_angle = 0.4;
      config.data.gait_step_duration = 800;
    }else if (buttnum == 4 && pressed) {
      config.data.gait_contact_angle = 0.5;
      config.data.gait_step_duration = 800;
    }
  }
}


