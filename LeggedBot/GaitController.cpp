#include <stdint.h>
#include "GaitController.h"


/**
   @brief Update walk movements. This should be called as frequently as possible.
   @param Current time, in milliseconds
*/
void GaitController::update(uint32_t time) {
  return;
}

/**
   @brief Assign setpoints for our overall linear and angular velocities.
   @param linear Linear velocity setpoint (m/sec, x axis)
   @param angular Angular velocity setpoint (rad/sec, z axis)
*/
void GaitController::setTwist(float linear, float angular) {
  return;
}

