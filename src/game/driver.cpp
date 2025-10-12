#include "game/driver.h"

void driver::checkInputs() {

  leftDrive.setVelocity(leftJoystick.getValue(), pct);
  rightDrive.setVelocity(rightJoystick.getValue(), pct);
  //leftDrive.setVelocity(curves::quadratic(Controller1.Axis3.position()), pct);
  //rightDrive.setVelocity(curves::quadratic(Controller1.Axis2.position()), pct);

  // Long Goal
  if (Controller1.ButtonR1.pressing()) {
    intakeLower.spin(fwd, 100, pct);
    intakeBack.spin(fwd, 100, pct);
    intakeUpper.spin(fwd, 100, pct);

  // Center Goal
  } else if (Controller1.ButtonR2.pressing()) {
    intakeLower.spin(fwd, 100, pct);
    intakeBack.spin(fwd, 100, pct);
    intakeUpper.spin(reverse, 100, pct);
  
  } else if (Controller1.ButtonL1.pressing()) {
    intakeBack.spin(fwd, 100, pct);
    intakeUpper.spin(fwd, 100, pct);

  } else if (Controller1.ButtonL2.pressing()) {
    intakeBack.spin(reverse, 100, pct);
    intakeUpper.spin(reverse, 100, pct);

  } else {
    intakeLower.stop(coast);
    intakeBack.stop(coast);
    intakeUpper.stop(coast);
  }
}