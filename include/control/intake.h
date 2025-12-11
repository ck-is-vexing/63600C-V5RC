#pragma once

#include "vex.h"
#include "robot-config.h"

/**
 * @namespace intake
 * @brief Intake-related control
 * 
 */
namespace intake {
  
  /// Changes based on whether the intake motors are running
  extern bool isActive;

  /// Changes based on whether the intake is currently preloading
  extern bool isPreloading;

  /**
   * @brief Load blocks up the intake for scoring
   * 
   */
  void preload();

  /**
   * @brief Initialize automatic color-based block sorting
   * 
   */
  void initSorting();

  /**
   * @brief Spin the intake to score in the long goal
   * 
   * @param speedPercent Intake motor speed percentage
   */
  void scoreLongGoal(const int speedPercent);

  /**
   * @brief Spin the intake to score in the center goal
   * 
   * @param generalSpeedPercent Intake motor general speed percentage
   * @param topSpeedPercent     Set slower than generalSpeedPercent to not shoot blocks through the middle goal
   * @param toggleMatchLoader   True will lower the match loader (so it doesn't get in the way)
   */
  void scoreCenterGoal(const int generalSpeedPercent, const int topSpeedPercent, const bool toggleMatchLoader = false);

  /**
   * @brief Spin the intake to score in the low goal
   * 
   * @param speedPercent Intake motor speed percentage
   */
  void scoreLowGoal(const int speedPercent);

  /**
   * @brief Spin the intake to store blocks in the hopper
   * 
   * @param speedPercent Intake motor speed percentage
   */
  void store(const int speedPercent);

  /**
   * @brief Spin the intake so blocks exit out the bottom
   * 
   * @param speedPercent Intake motor speed percentage
   */
  void outtake(const int speedPercent);

  /**
   * @brief Stop all intake motors
   * 
   * @param brakeType VEX brakeType (use coast typically)
   */
  void stop(const vex::brakeType brakeType);
}