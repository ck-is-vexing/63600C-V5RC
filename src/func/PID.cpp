#include "vex.h" // Include VEX headers

// Contains a general PID implementation to be used for more specific applications, such as turnTo
class PID {
  private:
    double Kp; // Proportional Tuning
    double Ki; // Integral Tuning
    double Kd; // Derivative Tuning
    double integral = 0; // Current integral value
    double derivative; // Current derivative value
    int dT; // The change in time, in ms, of every tick
    double oldError = 0; // The previous error, used for calculating Integral
  public:

    // Proportional, integral, and derivative tuning respectively
    // deltaTime should be the change in time, in ms, between ticks
    PID(double KProportional, double KIntegral, double KDerivative, int deltaTime)
    : Kp(KProportional), Ki(KIntegral), Kd(KDerivative), dT(deltaTime) {}

    // Returns the PID value for one tick
    // setpoint is the desired value for the variable to approach
    // pv is the process variable
    double update(double setpoint, double pv){ 

      // Update error
      // It's the difference between what is desired and what the current value is.
      double error = setpoint - pv;

      // Update integral
      // The integral calculates the area in between the graph of pv and the x-axis
      // To calculate integral, every tick the 
      integral += error * dT;

      // Update Derivative
      // Because the code only executes so often, the equation essentially is (y2 - y1) / (x2 - x1)
      // The y values are the current error, and the previous error
      // Because the change is always one tick, the denominator only has to be the length of one tick
      derivative = (error - oldError) / dT;

      // Update old error
      oldError = error;

      // Calculate the return value
      return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

    // Resets variables
    // Use when the setpoint changes
    void reset(){
      // Reset variables
      integral = 0;
    }
};