#include "control/ccurve.h"

// See https://www.desmos.com/calculator/mynqej9wdqs
namespace curves {
  double linear(int axisPos) {
    return axisPos;
  }

  double quadratic(int axisPos) {
    return 0.01 * axisPos * axisPos;
  }

  double exponential(int axisPos) {
    return copysign(pow(1.048, abs(axisPos)) - 1, axisPos);
  }

  double sigmoidal(int axisPos) {

    double k = 0.08; // scale of sigmoid
    double midpoint = 50;

    double f0 =   1 / (1 + exp(-k * (0 - midpoint)));
    double f100 = 1 / (1 + exp(-k * (100 - midpoint)));
    double fx =   1 / (1 + exp(-k * (axisPos - midpoint)));

    return copysign(100 * (fx - f0) / (f100 - f0), axisPos);
  }

  double sqrtSigmoidal(int axisPos) {
    if (abs(axisPos) < 50) {
      return copysign(-sqrt(-50 * (abs(axisPos - 50))) + 50, axisPos);
    } else {
      return copysign(7.07106781187 * sqrt(abs(axisPos) - 50) + 50, axisPos);
    }
  }
}

// RapidTrigger constructor
RapidTrigger::RapidTrigger(const vex::controller::axis& axis, 
                           std::function<double(int)> curve, 
                           int sensitivity)
: ax(axis), cur(curve), sens(sensitivity), last(0) {}

// Method
double RapidTrigger::getValue() {
  const int pos = ax.position();
  const unsigned int apos = abs(pos);
  double r;

  if ((last - sens) > apos) {
    r = 0;
  } else if ((last + sens) < apos) {
    r = copysign(100, pos);
  } else {
    r = cur(pos);
  }

  std::cout << pos << "    " << r << std::endl;
  last = apos;
  return r;
}