#pragma once

/**
 * @namespace global
 * @brief Global variables
 * 
 */
namespace global {

  /// Enables certain options for testing
  extern const bool debugMode;

  /// Declare whether the field has GPS strips
  extern const bool gpsAllowed;

  /// Switch between different autonomous options
  enum class autonomousTypes {
    NONE,
    SKILLS,
    LEFT,
    RIGHT,
    WINPOINT,
    TWO_INCH
  };
}