#include "control/color-sensor.h"
#include "definition.h"

ColorSensor::ColorSensor(const int32_t port) : vex::optical(port) {}

colorType ColorSensor::getBlock() {
  printl(isNearObject());
  //getRgb()

  return colorType::NONE;
}