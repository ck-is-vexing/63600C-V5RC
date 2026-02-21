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
  Pose calcPoseGPS();

  /**
   * @brief Get the robot pose using distance sensors and IMU
   * 
   * theta is in radians
   * (x, y) are in inches with (0, 0) being the center of the field
   * 
   * @return Pose 
   */
  Pose calcPoseDist();

  
  /**
   * @namespace odom 
   * @brief Methods for controlling and interpreting odometry
   * 
   */
  namespace odom {

    /**
     * @brief Get the robot pose using Odom and IMU
     * 
     * theta is in radians
     * (x, y) are in inches with (0, 0) being the center of the field
     * 
     * @return Pose 
     */
    const Pose getPose();

    /**
     * @brief Initialize background odometry calculations
     * 
     */
    void initTicker();

    /**
     * @brief Kill all odometry threads
     * 
     */
    void killTicker();
  }


  /**
   * @namespace render
   * @brief Methods to render the robot pose on Brain screen
   * 
   */
  namespace render {

    /**
     * @brief Initialize a thread running renderRobot
     * 
     */
    void initTicker();

    /**
     * @brief Kill all render threads
     * 
     */
    void killTicker();

    /**
     * @brief Renders an approximation of the robot position on Brain screen
     * 
     */
    void renderRobot();
  }
}