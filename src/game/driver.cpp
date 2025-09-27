#include "game/driver.h"

void driver::checkInputs() {

  leftDrive.setVelocity(leftJoystick.getValue(), pct);
  rightDrive.setVelocity(rightJoystick.getValue(), pct);

  if (Controller1.ButtonR1.pressing()) {
    intakeLower.spin(fwd, 100, pct);
    intakeUpper.spin(fwd, 100, pct);
    intakeBack.spin(fwd, 100, pct);
  } else {
    intakeLower.stop(coast);
    intakeUpper.stop(coast);
    intakeBack.stop(coast);
  }
}