#pragma once

/**
 * @namespace auton
 * @brief Contains functions that control the robot autonomously when run
 */
namespace auton {

  /**
   * @brief Left side game autonomous
   * 
   */
  void left();

  /**
   * @brief Right side game autonomous
   * 
   */
  void right();

  /**
   * @brief Solo winpoint
   * @warning Not complete!
   * 
   */
  void winpoint();

  /**
   * @brief Drive forwards 2 inches 
   * @note Used when our alliance partner has a solo winpoint
   * 
   */
  void twoInch();

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