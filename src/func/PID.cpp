#include "func/PID.h"

PID::PID(double Kp, double Ki, double Kd, int deltaTime, bool rotationFix)
: Kp(Kp), Ki(Ki), Kd(Kd), dT(deltaTime), rF(rotationFix) {}

double PID::update(double setpoint, double pv) { 

  double error = setpoint - pv;

  if (error > 180 && rF == true){
    error -= 360;
  } else if (error < -180 && rF == true){
    error += 360;
  }

  // Approximated every tick by adding new area to total sum
  integral += error * dT;

  // Derivative is approximated as (y2 - y1) / (x2 - x1)
  // Because the change is always one tick, the denominator only has to be the length of one tick
  derivative = (error - oldError) / dT;

  oldError = error;
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void PID::setTuning(double proportional, double integral, double derivative) {
  Kp = proportional;
  Ki = integral;
  Kd = derivative;
}

void PID::reset() {
  integral = 0;
}