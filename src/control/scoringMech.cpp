#include "control/scoringMech.h"
#include <iostream>

// TODO: Add comments!
wallStake::wallStake(vex::motor& motor, vex::rotation& rotationSensor)
: mot(motor), rot(rotationSensor), sp(0),
  wsPID(2, 0, 50, 20)
{
  rot.setPosition(0, vex::rotationUnits::deg);
}

void wallStake::setAngle(double angle) {
  sp = angle;
  running = true;
  wsPID.reset();
  mot.setVelocity(0, vex::percentUnits::pct);
  mot.spin(vex::directionType::rev);
}

double wallStake::angle(vex::rotationUnits unit) {
  return rot.position(unit);
}

void wallStake::tick() {
  if (running == true)
    mot.setVelocity(wsPID.update(sp, rot.position(vex::rotationUnits::deg)), vex::percentUnits::pct);
}

void wallStake::stop(vex::brakeType type) {
  mot.stop(type);
}

void wallStake::cancel() {
  running = false;
}