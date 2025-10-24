#include "game/autonomous.h"
#include "robot-config.h"

void auton::blueLeft() {
}

void auton::blueRight() {
}

void auton::redLeft() {
}

void auton::redRight() {
  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 30, 30);
  bot.turnTo(45);

  bot.drive(fwd, 9.5, 20);
  wait(0.5, sec);

  intakeLower.spin(reverse, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(reverse, 100, pct);
  hopper.spin(fwd, 100, pct);

  wait(2, sec);

  intakeLower.stop(coast);
  intakeBack.stop(coast);
  intakeUpper.stop(coast);
  hopper.stop(coast);

  bot.drive(reverse, 5);
  bot.turnTo(355);

  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 48, 40);

  wait(0.5, sec);

  bot.drive(reverse, 6, 30);

  bot.turnTo(135);
  bot.drive(fwd, 15.5, 30);

  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(reverse, 100, pct);
  hopper.spin(fwd, 100, pct);

  wait(3, sec);

  intakeLower.stop(coast);
  intakeBack.stop(coast);
  intakeUpper.stop(coast);
  hopper.stop(coast);
}

void auton::skills() {
}

void auton::PIDTest() {
  bot.turnTo(180);
  wait(2,sec);

  bot.turnTo(90);
  wait(2,sec);

  bot.turnTo(135);
  wait(2,sec);

  bot.turnTo(0);
  wait(2,sec);

  bot.turnTo(180);
  wait(2,sec);
}