#pragma once

#include "global.h"

/**
 * @namespace preAuton
 * @brief Functions for before the start of competition
 * 
 */
namespace preAuton {

  /**
   * @brief Modify the robot position based on GPS strip setup
   * @note sideAngle should be the GPS strip angle of the side the robot starts on
   * 
   */
  struct StartModifier {
    int x;
    int y;
    unsigned int sideAngle;
    bool flip;

    StartModifier();

    StartModifier(int x, int y, int sideAngle, bool flip);
  };
  
  /// Instance of StartModifier for modifying initial gps pose
  extern StartModifier startingGPS;

  /// enum instance for selecting auton
  extern autonomousTypes autonSelection;

  /// Starting angle of the robot. Used when GPS is disabled
  extern double inertialAngle;

  /**
   * @brief Load interface for autonomous selection
   * 
   */
  void autonSelector();

  /**
   * @brief Calibrate imu and odom based on GPS data
   * 
   * On average, more precise than setting up the robot perfectly every time.
   * GPS data is averaged over a period of time to cancel out noise.
   *
   * @param averageSeconds  # of seconds to average GPS data over
   */
  void sensorCalibration(double averageSeconds = 1);
}