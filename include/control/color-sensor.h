#pragma once

#include "vex.h"
#include "global.h"

/**
 * @brief Initialize a color sensor extending the VEX optical sensor
 * 
 */
class ColorSensor : public vex::optical {
  public:

    /**
     * @brief Construct a new Color Sensor 
     * 
     * @param port VEX brain Smart Port
     */
    ColorSensor(const int32_t port);

    void tick();

    colorType getBlock();
};