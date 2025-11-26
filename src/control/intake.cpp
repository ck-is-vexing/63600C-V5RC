#include "control/intake.h"

void intake::scoreLongGoal(const int speedPercent) {
  redirect.setTo(false);

  intakeLower.spin(fwd, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  intakeUpper.spin(fwd, speedPercent, pct);
  hopper.spin(fwd, speedPercent, pct);
}

void intake::scoreCenterGoal(const int generalSpeedPercent, const int topSpeedPercent, const bool toggleMatchLoader = false) {

  if (toggleMatchLoader) {
    matchLoadMech.toggle();
  }

  intakeLower.spin(fwd, generalSpeedPercent, pct);
  intakeBack.spin(fwd, generalSpeedPercent, pct);
  intakeUpper.spin(reverse, topSpeedPercent, pct);
  hopper.spin(fwd, generalSpeedPercent, pct);
}

void intake::scoreLowGoal(const int speedPercent) {
  intakeLower.spin(reverse, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  hopper.spin(fwd, speedPercent, pct);
}

void intake::store(const int speedPercent) {
  redirect.setTo(true);

  intakeLower.spin(fwd, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  intakeUpper.spin(fwd, speedPercent, pct);
}

void intake::outtake(const int speedPercent) {
  intakeLower.spin(reverse, speedPercent, pct);
  intakeBack.spin(reverse, speedPercent, pct);
  intakeUpper.spin(reverse, speedPercent, pct);
}

void intake::stop(const vex::brakeType brakeType) {
  intakeLower.stop(brakeType);
  intakeBack.stop(brakeType);
  intakeUpper.stop(brakeType);
  hopper.stop(brakeType);
}