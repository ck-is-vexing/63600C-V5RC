#include "game/driver.h"
#include "robot-config.h"

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
    hopper.spin(fwd, 100, pct);

    redirect.setTo(false);

  // Center Goal
  } else if (Controller1.ButtonR2.pressing()) {
    intakeLower.spin(fwd, 100, pct);
    intakeBack.spin(fwd, 100, pct);
    intakeUpper.spin(reverse, 100, pct);
    hopper.spin(fwd, 100, pct);
  
  // Intake into Hopper
  } else if (Controller1.ButtonL1.pressing()) {
    intakeLower.spin(fwd, 100, pct);
    intakeBack.spin(fwd, 100, pct);
    intakeUpper.spin(fwd, 100, pct);

    redirect.setTo(true);

  // Low Goal
  } else if (Controller1.ButtonL2.pressing()) {
    intakeLower.spin(reverse, 100, pct);
    intakeBack.spin(fwd, 100, pct);
    hopper.spin(fwd, 100, pct);
    
  // Outtake
  } else if (Controller1.ButtonY.pressing()) {
    intakeLower.spin(reverse, 100, pct);
    intakeBack.spin(reverse, 100, pct);
    intakeUpper.spin(reverse, 100, pct);

  } else {
    intakeLower.stop(coast);
    intakeBack.stop(coast);
    intakeUpper.stop(coast);
    hopper.stop(coast);
  }

  if (Controller1.ButtonRight.pressing()) {
    redirect.toggle(true);
  }

  if (Controller1.ButtonB.pressing()) {
    matchLoadMech.toggle();
  }
}