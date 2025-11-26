#include "control/rapid-trigger.h"

RapidTrigger::RapidTrigger(const vex::controller::axis& axis, 
                           std::function<double(int)> curve, 
                           int sensitivity)
: ax(axis), cur(curve), sens(sensitivity), last(0) {}

double RapidTrigger::calculateValue() {
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