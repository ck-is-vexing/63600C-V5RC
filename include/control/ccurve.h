#ifndef CCURVE_H
#define CCURVE_H

#include "vex.h"
#include <functional>
#include <cmath>
#include <iostream>

// Drive controller curves
namespace curves {
  double linear(int axisPos);
  double quadCurve(int axisPos);
  double expCurve(int axisPos);
  double expSCurve(int axisPos);
  double sqrtSCurve(int axisPos);
}


class RapidTrigger {
  private:
    const vex::controller::axis& ax;
    std::function<double(int)> cur; 
    unsigned int sens;
    unsigned int last;
  public:
    RapidTrigger(const vex::controller::axis& axis, 
                 std::function<double(int)> curve = std::function<double(int)>(curves::linear), 
                 int sensitivity = 0);

    double getValue();
};

#endif