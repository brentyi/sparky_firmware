#include <stdint.h>
#include "Configuration.h"
#include <EEPROM.h>

Configuration::Configuration() {
  return;
}

/**
   @brief Pull in saved configuration.
*/
void Configuration::pull() {
  EEPROM.get(0, data);
}

/**
   @brief Push current configuration.
*/
void Configuration::push() {
  EEPROM.put(0, data);
}
