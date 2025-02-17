#include "vex.h"
#include "func/PID.h"

// Assign Values
PID::PID(double KProportional, double KIntegral, double KDerivative, int deltaTime, bool rotationFix)
: Kp(KProportional), Ki(KIntegral), Kd(KDerivative), dT(deltaTime), rF(rotationFix) {}

// Update the PID controller and return the value to be applied to the mechanical system
double PID::update(double setpoint, double pv){ 

  // Update error
  // Error is the difference between what is desired and what the current value is
  double error = setpoint - pv;

  // Apply rotationFix
  if (error > 180 && rF == true){
    error -= 360;
  } else if (error < -180 && rF == true){
    error += 360;
  }

  // Update integral
  // The integral calculates the area in between the graph of pv and the t-axis (x-axis)
  // To calculate integral, every tick the new area is added to the total sum
  integral += error * dT;

  // Update Derivative (approximation)
  // Because the code only executes so often, the equation essentially is (y2 - y1) / (x2 - x1)
  // The y values are the current error and the previous error
  // Because the change is always one tick, the denominator only has to be the length of one tick
  derivative = (error - oldError) / dT;

  // Update old error
  oldError = error;

  // Calculate the return value
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// Resets values of the PID, to be used when the setpoint changes
void PID::reset(){
  integral = 0;
}