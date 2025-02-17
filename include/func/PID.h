#ifndef PID_H
#define PID_H

/**
 * @class PID
 * @brief A general implementation of a PID controller
 * 
 * This class contains a general Proportional-Integral-Derivative controller implementation,
 * suitable for controlling mechanical systems that require precise and quick adjustment.
 * The PID controller adjusts its output based on the error between
 * the desired setpoint and the current process variable.
 * 
 * Example usage: turnTo function in func/drivetrain.cpp
 */
class PID {
  private:
    double Kp; // Proportional Tuning
    double Ki; // Integral Tuning
    double Kd; // Derivative Tuning
    double integral = 0; // Current integral value
    double derivative; // Current derivative value
    int dT; // The change in time, in ms, of every tick
    bool rF; // Toggle for the rotational error fix
  public:
    double oldError; // The previous error, used for calculating Integral

     /**
     * @brief Constructs a PID controller with specified tuning parameters
     * 
     * Initializes the PID controller with the given proportional, integral, and derivative tuning values,
     * as well as the amount of time between steps (deltaTime). The rotationFix argument should be set to true 
     * when the process variable is going to be rotating (values between -360 and 360).
     * 
     * @param KProportional Proportional tuning value
     * @param KIntegral Integral tuning value
     * @param KDerivative Derivative tuning value
     * @param deltaTime The time that elapses between ticks
     * @param rotationFix (Optional) When true, rotationFix enables the outputted value to be in the correct direction for a rotating mechanical system
     */
    PID(double KProportional, double KIntegral, double KDerivative, int deltaTime, bool rotationFix = false);

    /**
     * @brief Updates the PID controller and retuns the value to be applied to the mechanical system
     * @param setpoint The desired value for the process variable to reach
     * @param pv Short for process variable, this is the current value of the variable that is intended to reach the setpoint
     * @return Returns a double type value to be applied to the mechanical system in order to approach the setpoint
     */
    double update(double setpoint, double pv);

    /**
     * @brief Resets values of the PID, to be used when the setpoint changes
     */
    void reset();
};

#endif