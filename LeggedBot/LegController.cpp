#include <stdint.h>
#include "LegController.h"

LegController::LegController(ConfigData* config, LegSideX x, LegSideY y) {
   
}

/**
 * @brief Update leg controls. This should be called as frequently as possible.
 * @returns Control effort, -1.0 to 1.0
 */
float LegController::calculateEffort() {
  return 0;
}

/**
 * @brief Set a desired leg position, at a given time.
 * @param destination Goal position, in radians.
 * @param arrival_time Expected arrival time for our goal position, in milliseconds.
 * @param backwards Use backwards leg movement.
 */
void LegController::setGoal(float destination, uint32_t arrival_time, bool backwards) {
  return;
}

