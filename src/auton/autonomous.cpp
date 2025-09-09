#include "auton/autonomous.h"
#include "robot-config.h"

namespace auton {

  void blueLeft() {
  }

  void blueRight() {
  }

  void redLeft() {

  }

  void redRight() {

  }

  void skills() {

  }

  // Test a couple different turnTo PID angles, used for tuning and debugging
  void PIDTest() {
    // Test different angles
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
}