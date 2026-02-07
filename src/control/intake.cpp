#include "control/intake.h"
#include "definition.h"
#include "global.h"

bool intake::isActive = false;

using namespace vex;

namespace {
  
  bool isPreloading = false;
  bool isSorting = false;

  /// @brief Thread function which detects the end of preloading blocks into the intake
  int preloadChecker() {
    printl("Preload Thread Init");

    while (true) {

      if (preloadColor.isNearObject()) {
        intakeLower.stop(brake);
        intakeBack.stop(brake);
        intakeUpper.stop(brake);
        hopper.stop(brake);

        this_thread::sleep_for(100);
        isPreloading = false;
        break;
      }

      // Failsafe exit
      if (Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing() || Controller1.ButtonR1.pressing() || Controller1.ButtonR2.pressing()) {
        intakeLower.stop(brake);
        intakeBack.stop(brake);
        intakeUpper.stop(brake);
        hopper.stop(brake);
        
        isPreloading = false;
        break;
      }

      this_thread::sleep_for(25);
    }

    printl("Thread Exiting!");
    return 0;
  }

  /// @brief Thread function which adjusts the intake according to the color of blocks traveling through it
  int colorSort() {
    printl("Color Sort Thread Init");

    colorType colorToRemove;

    if (global::yourColor == colorType::BLUE) {
      colorToRemove = colorType::RED;
    } else {
      colorToRemove = colorType::BLUE;
    }

    while (true) {

      if (intake::isActive && sortColor.getBlock() == colorToRemove && !isPreloading) {
        isSorting = true;

        redirect.setTo(true);
        this_thread::sleep_for(250);
        redirect.setTo(false);

        isSorting = false;
      }
      
      // Sort cancellation
      if (Controller2.ButtonA.pressing()) { break; }

      this_thread::sleep_for(25);
    }

    return 0;
  }
}

void intake::preload() {
  isPreloading = true;

  intakeLower.spin(fwd, 10, pct);
  intakeBack.spin(fwd, 15, pct);
  intakeUpper.spin(fwd, 10, pct);
  hopper.spin(fwd, 100, pct);

  thread preloadCheckerThread = thread(preloadChecker);
}

void intake::initSorting() {
  thread colorSortThread = thread(colorSort);
}

void intake::scoreLongGoal(const int speedPercent) {

  if (!isSorting) {
    redirect.setTo(false);
  }

  intakeLower.spin(fwd, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  intakeUpper.spin(fwd, speedPercent, pct);
  hopper.spin(fwd, 100, pct);

  intake::isActive = true;
}

void intake::scoreCenterGoal(const int generalSpeedPercent, const int topSpeedPercent, const bool toggleMatchLoader) {

  if (toggleMatchLoader) {
    matchLoadMech.toggle();
  }

  intakeLower.spin(fwd, generalSpeedPercent, pct);
  intakeBack.spin(fwd, generalSpeedPercent, pct);
  intakeUpper.spin(reverse, topSpeedPercent, pct);
  hopper.spin(fwd, 100, pct);

  intake::isActive = true;
}

void intake::scoreLowGoal(const int speedPercent) {
  intakeLower.spin(reverse, speedPercent, pct);
  intakeBack.spin(fwd, speedPercent, pct);
  hopper.spin(fwd, 100, pct);

  intake::isActive = true;
}

void intake::store(const int speedPercent) {
  if (!isSorting) {
    redirect.setTo(true);
  }

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