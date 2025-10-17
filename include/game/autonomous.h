#pragma once

/**
 * @namespace auton
 * @brief Contains functions that control the robot autonomously when run
 */
namespace auton {

  /**
   * @brief Blue left side game autonomous
   * 
   */
  void blueLeft();

  /**
   * @brief Blue right side game autonomous
   * 
   */
  void blueRight();

  /**
   * @brief Red left side game autonomous
   * 
   */
  void redLeft();

  /**
   * @brief Red right side game autonomous
   * 
   */
  void redRight();

  /**
   * @brief Skills competition autonomous
   *
   */
  void skills();

  /**
   * @brief Test a couple different turnTo PID angles
   * @note Used for tuning and debugging
   *
   */
  void PIDTest();
}