#include "game/driver.h"
#include "robot-config.h"
#include "control/intake.h"

double speedModifier = 1.0;
int alignerTicker = 0;

namespace {
  constexpr int MAX_ALIGNER_SECONDS = 2;
  constexpr int MAIN_LOOP_LENGTH_MS = 20;
  constexpr int MAX_ALIGNER_TIME    = MAX_ALIGNER_SECONDS * (1000 / MAIN_LOOP_LENGTH_MS);
}

void driver::registerEvents() {

  // Primary controller
  Controller1.ButtonY.pressed(     []() { matchLoadMech.toggle(true);  aligner.setTo(false);          });
  Controller1.ButtonB.pressed(     []() { intake::preload();           aligner.setTo(true);           }); 

  Controller1.ButtonLeft.pressed(  []() { redirect.toggle(true);                                      });
  Controller1.ButtonRight.pressed( []() { wing.toggle(true);                                          });

  // Secondary controller
  Controller2.ButtonL1.pressed(    []() { global::yourColor = colorType::BLUE; intake::initSorting(); });
  Controller2.ButtonL2.pressed(    []() { global::yourColor = colorType::BLUE; intake::initSorting(); });
  Controller2.ButtonR1.pressed(    []() { global::yourColor = colorType::RED;  intake::initSorting(); });
  Controller2.ButtonR2.pressed(    []() { global::yourColor = colorType::RED;  intake::initSorting(); });
}

void driver::checkInputs() {

  leftDrive.setVelocity(  leftJoystick.calculateValue()  * speedModifier, pct);
  rightDrive.setVelocity( rightJoystick.calculateValue() * speedModifier, pct);


  if (Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()) {
    if (aligner.getValue()) {
      aligner.setTo(true);
    } else {
      aligner.setTo(false);
    }

  } else if (Controller1.ButtonR1.pressing()) {
    intake::scoreLongGoal(100 * speedModifier);
    alignerTicker = 0;
    aligner.setTo(true);

  } else if (Controller1.ButtonR2.pressing()) {
    intake::scoreCenterGoal(100 * speedModifier, 50 * speedModifier);
    alignerTicker = 0;
    aligner.setTo(true);
  
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


  if ((alignerTicker > MAX_ALIGNER_TIME) && !intake::isPreloading) {
    aligner.setTo(false);
  }

  alignerTicker++;
}