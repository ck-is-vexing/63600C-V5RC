#include "game/driver.h"
#include "robot-config.h"
#include "control/intake.h"

void driver::registerEvents() {
  Controller1.ButtonY.pressed(     []() { matchLoadMech.toggle(true); });
  Controller1.ButtonDown.pressed(  []() { redirect.toggle(true);      });
  Controller1.ButtonRight.pressed( []() { wing.toggle(true);          });
  Controller1.ButtonB.pressed(     []() { intake::preload();          });
}

void driver::checkInputs() {

  leftDrive.setVelocity(  leftJoystick.calculateValue(),  pct);
  rightDrive.setVelocity( rightJoystick.calculateValue(), pct);

  if        (Controller1.ButtonR1.pressing()) {
    intake::scoreLongGoal(100);

  } else if (Controller1.ButtonR2.pressing()) {
    intake::scoreCenterGoal(100, 50);
  
  } else if (Controller1.ButtonL1.pressing()) {
    intake::store(100);

  } else if (Controller1.ButtonL2.pressing()) {
    intake::scoreLowGoal(100);

  } else if (Controller1.ButtonA.pressing() ) {
    intake::outtake(100);

  } else {
    intake::stop(coast);
  }
}