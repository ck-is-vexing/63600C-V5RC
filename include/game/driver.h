#pragma once

/**
 * @namespace driver
 * @brief Contains functions used during driver portion of a match
 * 
 */
namespace driver {

  /**
   * @brief Register driver functions called on button presses
   * 
   */
  void registerEvents();

  /**
   * @brief Check controller inputs and respond accordingly
   * 
   */
  void checkInputs();
}