#pragma once

/**
 * @namespace pose
 * @brief Methods to locate the robot on the field
 * 
 */
namespace pose {

  /**
   * @brief Robot Pose struct
   * (x, y, theta)
   * 
   */
  struct Pose {
    double x;
    double y;
    double theta;

    Pose();

    Pose(double x, double y, double theta);
  };
  
  /// Starting position of robot
  extern Pose startingPose;

  /**
   * @brief Get the robot pose using GPS and IMU
   * 
   * theta is in radians
   * (x, y) are in inches with (0, 0) being the center of the field
   * 
   * @return Pose 
   */
  Pose getPoseGPS();

  /**
   * @brief Get the robot pose using Odom and IMU
   * 
   * theta is in radians
   * (x, y) are in inches with (0, 0) being the center of the field
   * 
   * @return Pose 
   */
  Pose getPoseOdom();

  /**
   * @brief Renders an approximation of the robot position on Brain screen
   * 
   */
  void renderRobot();
}