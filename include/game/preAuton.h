#pragma once

#include "robot-config.h"

/**
 * @brief Functions for before the start of competition
 * 
 */
namespace preAuton {

  /**
   * @brief Declare placement of GPS strips based on blue side
   * @note Should be 90 for a correctly set up field 
   *
   */
  extern int gpsBlueAngle;
  
  /**
   * @brief Load interface for autonomous selection
   * 
   */
  void autonSelector();

  /**
   * @brief Calibrate inertial sensor based on GPS angle of robot
   * 
   * On average, more precise than setting up the robot perfectly every time.
   * GPS data is averaged over a period of time to cancel out noise.
   *
   * @param averageSeconds # of seconds to average GPS data over
   */
  void inertialGPSCalibrate(double averageSeconds = 1);

  //void autonSelector();
}