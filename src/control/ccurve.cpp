#include "control/ccurve.h"


double curves::k = 0.08;

double curves::linear(int axisPos) {
  return axisPos;
}

double curves::quadratic(int axisPos) {
  return copysign( 0.01 * axisPos * axisPos , axisPos);
}

double curves::exponential(int axisPos) {
  return copysign( pow(1.048, abs(axisPos)) - 1 , axisPos);
}

double curves::sigmoidal(int axisPos) {
  return copysign( 103.7315 / (1 + exp(-curves::k * (abs(axisPos) - 50))) - 1.8657 , axisPos);
}

double curves::sqrtSigmoidal(int axisPos) {
  if (abs(axisPos) < 50) {
    return copysign( -sqrt(-50 * (abs(axisPos - 50))) + 50 , axisPos);
  } else {
    return copysign( 7.0711 * sqrt(abs(axisPos) - 50) + 50 , axisPos);
  }
}


RapidTrigger::RapidTrigger(const vex::controller::axis& axis, 
                           std::function<double(int)> curve, 
                           int sensitivity)
: ax(axis), cur(curve), sens(sensitivity), last(0) {}

double RapidTrigger::getValue() {
  const int pos = ax.position();
  const int apos = abs(pos);
  double r;

  if ((last - sens) > apos) {
    r = 0;
  } else if ((last + sens) < apos) {
    r = copysign(100, pos);
  } else {
    r = cur(pos);
  }

  last = apos;
  return r;
}