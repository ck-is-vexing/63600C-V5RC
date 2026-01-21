#pragma once

/// Switch between different autonomous options
enum class autonomousTypes {
  NONE,
  SKILLS,
  LEFT,
  RIGHT,
  WINPOINT,
  TWO_INCH
};

/**
 * @brief Contains VEX team colors
 * 
 */
enum class colorType {
  NONE,
  RED,
  BLUE
};

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

  /// Declare your alliance's color
  extern colorType  yourColor;
}