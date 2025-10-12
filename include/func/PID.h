#pragma once

#include "vex.h"

/**
 * @class PID
 * @brief A general implementation of a PID controller
 * 
 * This class contains a general Proportional-Integral-Derivative controller,
 * suitable for controlling mechanical systems that require precise and quick adjustment.
 * The PID controller adjusts its output based on the error between
 * the desired setpoint and the current process variable.
 * 
 * @example Example usage: turnTo function in func/drivetrain.cpp
 *
 */
class PID {
  private:
    double Kp;
    double Ki;
    double Kd;
    double integral = 0;
    double derivative;
    int dT;
    bool rF; // Toggle for rotational error fix
  public:
    double oldError; // The previous error, used for calculating Integral

     /**
     * @brief Construct a PID controller with specified tuning parameters
     * 
     * Initialize the PID controller with the given proportional, integral, and derivative tuning values,
     * as well as the amount of time between steps (deltaTime). The rotationFix argument should be set to true 
     * when the process variable is going to be rotating (values between -360 and 360).
     * 
     * @param Kp Proportional tuning value
     * @param Ki Integral tuning value
     * @param Kd Derivative tuning value
     * @param deltaTime The time that elapses between ticks, in ms
     * @param rotationFix (Optional) When true, rotationFix enables the outputted value to be in the correct direction for a 360 degree mechanical system
     * 
     */
    PID(double Kp, double Ki, double Kd, int deltaTime, bool rotationFix = false);

    /**
     * @brief Updates the PID controller and retuns the value to be applied to the mechanical system
     *
     * @param setpoint The desired value for the process variable to reach
     * @param pv Short for process variable, this is the current value of the variable that is intended to reach the setpoint
     * @return Returns a double type value to be applied to the mechanical system in order to approach the setpoint
     *
     */
    double update(double setpoint, double pv);

    /**
     * @brief Resets values of the PID Controller, use when setpoint changes
     * 
     */
    void reset();
};