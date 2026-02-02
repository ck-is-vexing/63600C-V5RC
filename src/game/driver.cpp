#include "game/driver.h"
#include "robot-config.h"
#include "control/intake.h"

double speedModifier = 1.0;

void driver::registerEvents() {

  // Primary controller
  Controller1.ButtonY.pressed(     []() { matchLoadMech.toggle(true); });
  Controller1.ButtonLeft.pressed(  []() { redirect.toggle(true);      });
  Controller1.ButtonRight.pressed( []() { wing.toggle(true);          });
  Controller1.ButtonB.pressed(     []() { intake::preload();          });

  // Secondary controller
  Controller2.ButtonL1.pressed(    []() { global::yourColor = colorType::BLUE; intake::initSorting(); });
  Controller2.ButtonL2.pressed(    []() { global::yourColor = colorType::BLUE; intake::initSorting(); });
  Controller2.ButtonR1.pressed(    []() { global::yourColor = colorType::RED;  intake::initSorting(); });
  Controller2.ButtonR2.pressed(    []() { global::yourColor = colorType::RED;  intake::initSorting(); });
}

void driver::checkInputs() {

  leftDrive.setVelocity(  leftJoystick.calculateValue()  * speedModifier, pct);
  rightDrive.setVelocity( rightJoystick.calculateValue() * speedModifier, pct);

  if        (Controller1.ButtonR1.pressing()) {
    intake::scoreLongGoal(100 * speedModifier);

  } else if (Controller1.ButtonR2.pressing()) {
    intake::scoreCenterGoal(100 * speedModifier, 50 * speedModifier);
  
  } else if (Controller1.ButtonL1.pressing()) {
    intake::store(100);

  } else if (Controller1.ButtonL2.pressing()) {
    intake::scoreLowGoal(100 * speedModifier);

  } else if (Controller1.ButtonA.pressing() ) {
    intake::outtake(100 * speedModifier);

  } else {
    intake::stop(coast);
  }

  if (Controller1.ButtonDown.pressing()) {
    speedModifier = 0.5;
  } else {
    speedModifier = 1;
  }
}