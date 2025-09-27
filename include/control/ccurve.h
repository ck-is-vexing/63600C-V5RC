#pragma once

#include "vex.h"
#include <functional>
#include <iostream>

/**
 * @namespace curves
 * @brief Contains functions which transform VEX controller axis data
 * 
 * Functions take in VEX controller axis data on a range of [-100, 100],
 * apply a mathematical transformation, and return the result.
 * 
 * @note See https://www.desmos.com/calculator/5in1dplxfn for mathematical models of curves
 */
namespace curves {

  extern double k; // sigmoidal tuning, balance of notable but not too steep

  /**
   * @brief Returns axis position, use as a placeholder when no transformation is intended
   * 
   * f(x) = x
   *
   * @param axisPos Value of VEX controller axis with range of [-100, 100]
   * @return double
   */
  double linear(int axisPos);

  /**
   * @brief Quadratic transformation of axis position
   * 
   * Output changes quadratically with input, while preserving sign.
   * Use for slightly more control over small return values, with a little delay before returning high values
   * f(x) = ±0.01x^2
   *
   * @param axisPos Value of VEX controller axis with range of [-100, 100]
   * @return double, transformed axisPos
   */
  double quadratic(int axisPos);
  
  /**
   * @brief Exponential transformation of axis position
   * 
   * Output changes exponentially with input, while preserving sign.
   * Use for notably more control over small return values, except much delay before returning high values
   * f(x) = ±(1.048^x - 1)
   *
   * @param axisPos Value of VEX controller axis with range of [-100, 100]
   * @return double, transformed axisPos
   */
  double exponential(int axisPos);

  /**
   * @brief Sigmoidal transformation of axis position, adjusted to [-100, 100]
   * 
   * Applies a sigmoidal transformation to input axis position, with
   * the scale (k) being 0.08, adjusted to fix axis values.
   * See implementation for equation.
   *
   * @param axisPos Value of VEX controller axis with range of [-100, 100]
   * @return double, transformed axisPos
   */
  double sigmoidal(int axisPos);

  /**
   * @brief Sigmoidal-inspired transformation of axis position using sqrt
   * 
   * Transformes data based on two square roots, similar to a sigmoid:
   * For x < 50, f(x) = -√(-50(x - 50) - 50) + 50
   * For x ≥ 50, f(x) = 7.0711 * √(x - 50) + 50
   *
   * @param axisPos value of VEX controller axis with range of [-100, 100]
   * @return double, transformed axisPos
   */
  double sqrtSigmoidal(int axisPos);
}

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
    double getValue();
};