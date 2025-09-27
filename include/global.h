#pragma once

/**
 * @brief Global variables
 * 
 */
namespace global {

  /// Enables certain options for testing
  extern const bool debugMode;

  /// Declare whether the field has GPS strips
  extern const bool gpsAllowed;

  /// Switch between different autonomous options
  enum autonomousTypes {
    NONE,
    SKILLS,
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT
  };
}