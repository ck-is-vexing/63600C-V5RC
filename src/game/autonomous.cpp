#include "auton/autonomous.h"

void auton::blueLeft() {
}

void auton::blueRight() {
}

void auton::redLeft() {
}

void auton::redRight() {
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