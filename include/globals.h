#ifndef GLOBALS_H
#define GLOBALS_H

// TODO: add comments
namespace global {
  extern const bool debugMode;
  extern const bool gpsAllowed;

  enum class autonSwitch {
    none,
    testing,
    blueLeft,
    blueRight,
    redLeft,
    redRight,
    skills
  };
}

#endif