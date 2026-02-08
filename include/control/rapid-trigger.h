#pragma once

#include "vex.h"
#include "func/curves.h"
#include <functional>

/**
 * @brief Rapid Trigger snapping functionality for VEX Controller axes
 * 
 * Snaps value to the extremes of the axis while it is changing.
 * When the rate of change exceeds the sensitivity, if traveling away
 * from 0, the value of the RapidTrigger snaps to 100 or -100, based on
 * direction. Similarly, when the axis is moving towards 0 faster than
 * sensitivity, the value snaps to 0. If the axis isn't moving faster than
 * sensitivity, the RapidTrigger inherits the current axis value.
 */
class RapidTrigger {
  private:
    const vex::controller::axis& ax;
    std::function<double(int)> cur; 
    int sens;
    int last;

  public:

    /**
     * @brief Construct a RapidTrigger object
     * 
     * @param axis The axis to take as input
     * @param curve A custom curve to be applied to the output of the RapidTrigger
     * @param sensitivity Threshold that the rate of change must exceed to snap to an extreme
     */
    RapidTrigger(const vex::controller::axis& axis, 
                 std::function<double(int)> curve = std::function<double(int)>(curves::linear), 
                 int sensitivity = 5);
    
    /**
     * @brief Returns the current value of the RapidTrigger, based in parameters from class init
     * 
     * @return double 
     */
    double calculateValue();
};