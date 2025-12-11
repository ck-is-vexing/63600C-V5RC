#include "control/color-sensor.h"
#include "definition.h"
#include "vex.h"

using namespace vex;

ColorSensor::ColorSensor(const int32_t port) : vex::optical(port) {}

colorType ColorSensor::getBlock() {
  if (isNearObject()) {

    const vex::color detectedColor = color();

    if (detectedColor == vex::color::blue) {
      return colorType::BLUE;
    } else if (detectedColor == vex::color::red) {
      return colorType::RED;
    }
  }
  
  return colorType::NONE;
}