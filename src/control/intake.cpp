#include "control/intake.h"
#include "definition.h"

bool intake::isActive = false;
bool intake::isPreloading = false;

using namespace vex;

namespace {
  int endPreload() {
    printl("Thread Starting!");

    while (true) {

      if (intakeColor.isNearObject()) {
        
        intakeLower.stop(brake);
        intakeBack.stop(brake);
        intakeUpper.stop(brake);
        hopper.stop(brake);

        intakeColor.integrationTime(103);

        this_thread::sleep_for(100);
        intake::isPreloading = false;
        break;
      }

      this_thread::sleep_for(25);
    }

    printl("Thread Exiting!");
    return 0;
  }
}

void intake::preload() {
  intake::isPreloading = true;

  intakeColor.integrationTime(25);

  intakeLower.spin(fwd, 10, pct);
  intakeBack.spin(fwd, 15, pct);
  intakeUpper.spin(fwd, 10, pct);
  hopper.spin(fwd, 100, pct);

  thread preloadChecker = thread(endPreload);
}

void intake::scoreLongGoal(const int speedPercent) {
  redirect.setTo(false);

  intakeLower.spin(fwd, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  intakeUpper.spin(fwd, speedPercent, pct);
  hopper.spin(fwd, speedPercent, pct);

  intake::isActive = true;
}

void intake::scoreCenterGoal(const int generalSpeedPercent, const int topSpeedPercent, const bool toggleMatchLoader) {

  if (toggleMatchLoader) {
    matchLoadMech.toggle();
  }

  intakeLower.spin(fwd, generalSpeedPercent, pct);
  intakeBack.spin(fwd, generalSpeedPercent, pct);
  intakeUpper.spin(reverse, topSpeedPercent, pct);
  hopper.spin(fwd, generalSpeedPercent, pct);

  intake::isActive = true;
}

void intake::scoreLowGoal(const int speedPercent) {
  intakeLower.spin(reverse, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  hopper.spin(fwd, speedPercent, pct);

  intake::isActive = true;
}

void intake::store(const int speedPercent) {
  redirect.setTo(true);

  intakeLower.spin(fwd, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  intakeUpper.spin(fwd, speedPercent, pct);

  intake::isActive = true;
}

void intake::outtake(const int speedPercent) {
  intakeLower.spin(reverse, speedPercent, pct);
  intakeBack.spin(reverse, speedPercent, pct);
  intakeUpper.spin(reverse, speedPercent, pct);

  intake::isActive = true;
}

void intake::stop(const vex::brakeType brakeType) {

  if (!isPreloading) {
    intakeLower.stop(brakeType);
    intakeBack.stop(brakeType);
    intakeUpper.stop(brakeType);
    hopper.stop(brakeType);

    intake::isActive = false;
  }
}