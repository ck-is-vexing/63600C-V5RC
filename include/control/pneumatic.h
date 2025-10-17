#pragma once

#include "vex.h"

/**
 * @brief Initialize pneumatics and control them
 * 
 * Very similar to VEX library definitions. This requires a
 * port and optionally the ability to reverse the pneumatic.
 */
class Pneumatic {
  private:
    vex::digital_out solenoid;
    const bool reversed;
    uint32_t toggleCooldownTime;
  public:
  
    /**
     * @brief Construct a Pneumatic object
     * 
     * @param solenoidPort Pneumatic 3 wire port
     * @param reverse Set to true to reverse the pneumatic values
     */
    Pneumatic(vex::triport::port& solenoidPort, const bool reverse = false);

    /**
     * @brief Returns the current pneumatic value
     * 
     * @return true if extended
     * @return false if retracted
     */
    bool getValue();

    /**
     * @brief Toggle the pneumatic to the opposite state
     * 
     */
    void toggle();

    /**
     * @brief Set the pneumatic's state
     * 
     * @param value true means extended, false means retracted
     */
    void setTo(const bool value);
};