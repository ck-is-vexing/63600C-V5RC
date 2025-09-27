#pragma once

#include "robot-config.h"
#include "func/button.h"
#include "global.h"

/**
 * @namespace preAuton
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
   * @brief enum instance for selecting auton
   * 
   */
  extern global::autonomousTypes autonSelection;

  /**
   * @brief Starting angle of the robot. Used when GPS is disabled
   * 
   */
  extern int inertialAngle;

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
}