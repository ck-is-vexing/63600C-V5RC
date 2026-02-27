#include "game/driver.h"

#include "robot-config.h"
#include "control/intake.h"
#include "definition.h"

double speedModifier    = 1.0;
int alignerCooldownTime = 0;

namespace {
  constexpr int MAX_ALIGNER_SECONDS              = 1;
  constexpr int ALIGNER_FORCE_TOGGLE_COOLDOWN_MS = 1000;

  constexpr int MAIN_LOOP_LENGTH_MS              = 20;
  constexpr int MAX_ALIGNER_TIME                 = MAX_ALIGNER_SECONDS * (1000 / MAIN_LOOP_LENGTH_MS);
}

void driver::registerEvents() {

  // Primary controller
  Controller1.ButtonY.pressed(     []() { matchLoadMech.toggle(true);  aligner.setTo(false);          });
  Controller1.ButtonB.pressed(     []() { intake::preload();           aligner.setTo(true);           }); 

  Controller1.ButtonLeft.pressed(  []() { redirect.toggle(true);                                      });
  Controller1.ButtonRight.pressed( []() { wing.toggle(true);                                          });
  Controller1.ButtonUp.pressed(    []() { pose::Pose curPose = pose::odom::getPose(); printl(curPose.x << "    " << curPose.y << "    " << curPose.theta); });

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

    // Needs to be a separate logic check so the other intake functions aren't called unintentionally
    if ((alignerCooldownTime + ALIGNER_FORCE_TOGGLE_COOLDOWN_MS) < vex::timer::system()) {

      alignerCooldownTime = vex::timer::system();

      if (aligner.getValue()) {
        aligner.setTo(false);
      } else {
        aligner.setTo(true);
      }
    }

  } else if (Controller1.ButtonR1.pressing()) {
    intake::isPreloading  = false;
    intake::alignerTicker = 0;
    aligner.setTo(true);
    intake::scoreLongGoal(100 * speedModifier);

  } else if (Controller1.ButtonR2.pressing()) {
    intake::isPreloading  = false;
    intake::alignerTicker = 0;
    aligner.setTo(true);
    intake::scoreCenterGoal(100 * speedModifier, 50 * speedModifier);
  
  } else if (Controller1.ButtonL1.pressing()) {
    intake::isPreloading  = false;
    intake::store(100);

  } else if (Controller1.ButtonL2.pressing()) {
    intake::isPreloading  = false;
    intake::scoreLowGoal(50 * speedModifier); // 50 for driver skills program, 100 normally

  } else if (Controller1.ButtonA.pressing() ) {
    intake::isPreloading  = false;
    intake::outtake(100 * speedModifier);

  } else {
    intake::stop(coast);
  }

  speedModifier = (Controller1.ButtonDown.pressing()) ? 0.5 : 1;


  if ((intake::alignerTicker > MAX_ALIGNER_TIME) && !intake::isPreloading) {
    aligner.setTo(false);
  }

  intake::alignerTicker++;
}