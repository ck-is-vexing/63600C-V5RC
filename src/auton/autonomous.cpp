#include "auton/autonomous.h"
#include "robot-config.h"

namespace auton {

  void blueLeft() {
    // Drive backwards up to the goal
    leftDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);
    
    // Drive backwards to safety
    leftDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct);
    
    leftDrive.spinFor(fwd, 350, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 350, deg, 50, velocityUnits::pct);

    // Score?!!!!
    intakeUpper.spinFor(3,sec, 40, velocityUnits::pct);
    
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,50,pct);

    leftDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct);

    wait(3000,msec);

    leftDrive.spinFor(fwd, 1000, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 1000, deg, 50, velocityUnits::pct);

    leftDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct);
  }

  void blueRight() {
    // Drive backwards up to the goal
    leftDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);
    
    // Drive backwards to safety
    leftDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct);
    
    bot.turnTo(0,2);

    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,50,pct);

    wait(1,sec);

    leftDrive.spinFor(fwd, 1500, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1500, deg, 50, velocityUnits::pct);

    wait(1,sec);

    bot.turnTo(270,1);
    leftDrive.spinFor(fwd, 850, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 850, deg, 50, velocityUnits::pct);
    
    wait(3,sec);
  }

  void redLeft() {
    // Drive backwards up to the goal
    leftDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);

    // Drive backwards to safety
    leftDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct);

    bot.turnTo(0);

    // Score?!!!!
    intakeUpper.spinFor(3,sec, 40, velocityUnits::pct);

    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,50,pct);

    leftDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct);

    wait(3000,msec);

    bot.turnTo(180);

    leftDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct);
  }

  void redRight() {
    // Drive backwards up to the goal
    leftDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);
    
    // Drive backwards to safety
    leftDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 500, deg, 50, velocityUnits::pct);
    
    bot.turnTo(180);

    // Score?!!!!
    intakeUpper.spinFor(3,sec, 40, velocityUnits::pct);
    
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,50,pct);

    leftDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct);

    wait(3000,msec);

    bot.turnTo(0);

    leftDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct);
  }

  void skills() {
    // ************** FIRST CORNER **************
    // Drive up to goal
    leftDrive.spinFor(reverse, 500, deg, 20, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 500, deg, 20, velocityUnits::pct);

    // Clamp
    clampPneumatic.set(true);

    // Start intake
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    // Wait a bit
    wait(1000,msec);

    // Spin to face first ring
    bot.turnTo(90, 1.5);

    // Grab ring
    leftDrive.spinFor(fwd, 1600, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1600, deg, 50, velocityUnits::pct);

    // Spin to face second ring
    bot.turnTo(0, 2);

    // Grab ring
    leftDrive.spinFor(fwd, 1550, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1550, deg, 50, velocityUnits::pct);

    // Spin to third and fourth rings
    bot.turnTo(270, 1.5);

    // Grab rings
    leftDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct);

    // Spin to face final ring
    bot.turnTo(48, 2);

    // Pick up ring
    leftDrive.spinFor(fwd, 800, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 800, deg, 30, velocityUnits::pct);

    wait(1,sec);

    bot.turnTo(110, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.drive(reverse, 8, 20);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.drive(fwd, 7, 20);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    bot.turnTo(0.5,0.3);

    bot.drive(reverse, 69, 40);

    bot.drive(reverse, 9, 20);

    // Clamp
    clampPneumatic.set(true);

    bot.drive(reverse, 5, 20);

    wait(100, msec);

    // ************** SECOND CORNER **************
    
    // Spin to face first ring
    bot.turnTo(90, 1);

    // Start intake
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    // Grab ring
    bot.drive(fwd, 32, 30);

    // Face second ring
    bot.turnTo(155, 1);

    bot.drive(fwd, 40);

    bot.drive(reverse, 4);

    // Spin to face third ring
    bot.turnTo(285, 2);

    // Grab ring
    bot.drive(fwd, 23);

    // Spin to fourth and fifth rings
    bot.turnTo(270, 1);

    // Grab rings
    leftDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct);

    wait(0.5,sec);

    // Spin to face final ring
    bot.turnTo(135, 2);

    // Pick up ring
    bot.drive(fwd, 20);

    wait(1,sec);

    bot.turnTo(75, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.drive(reverse, 12, 22);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.drive(fwd, 5, 20);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    
    // ************** SECOND HALF **************

    bot.turnTo(0, 2);
    bot.drive(fwd, 15);

    // zoop under the bar!
    bot.turnTo(45, 0.8);
    leftDrive.setStopping(coast);
    rightDrive.setStopping(coast);
    intakeLower.spin(fwd,100,pct);
    bot.drive(fwd, 75);

    // First ring
    intakeUpper.spinFor(fwd, 800, deg, false);
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);

    // Second ring
    bot.drive(fwd, 34);
    intakeUpper.spinFor(fwd, 800, deg, false);

    // Goal
    bot.drive(reverse, 8);
    intakeLower.stop(coast);
    bot.turnTo(315, 2);

    bot.drive(reverse, 36);
    clampPneumatic.set(true);
    bot.drive(reverse, 2);

    // Third ring
    bot.turnTo(225, 2);
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);
    bot.drive(fwd, 35);

    // Spin to fourth ring
    bot.turnTo(135, 1.5);

    // Grab ring
    bot.drive(fwd, 31);

    // Spin to face final ring
    bot.turnTo(180, 2);

    // Pick up ring
    bot.drive(fwd, 14);

    wait(1,sec);

    bot.turnTo(290, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.drive(reverse, 10, 20);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.drive(fwd, 7, 40);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Slam into corner, just to be safe
    bot.drive(reverse, 10, 100);
    bot.drive(fwd, 7, 40);

    // Push final goal into corner
    bot.turnTo(15,2);
    intakeLower.spin(fwd,30,pct);
    bot.drive(fwd, 132, 100);
    intakeLower.spin(reverse,100,pct);

    wait(200, msec);

    //bot.turnTo(45, 5);
    //bot.drive(fwd, 30, 100);

    bot.drive(reverse, 50, 100);
    intakeLower.stop(coast);
  }

  void skillsNWS() {
    // ************** FIRST CORNER **************
    // Drive up to goal
    leftDrive.spinFor(reverse, 500, deg, 20, velocityUnits::pct, false);
    rightDrive.spinFor(reverse, 500, deg, 20, velocityUnits::pct);

    // Clamp
    clampPneumatic.set(true);

    // Start intake
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    // Wait a bit
    wait(1000,msec);

    // Spin to face first ring
    bot.turnTo(90, 1.5);

    // Grab ring
    leftDrive.spinFor(fwd, 1600, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1600, deg, 50, velocityUnits::pct);

    // Spin to face second ring
    bot.turnTo(0, 2);

    // Grab ring
    leftDrive.spinFor(fwd, 1550, deg, 50, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 1550, deg, 50, velocityUnits::pct);

    // Spin to third and fourth rings
    bot.turnTo(270, 1.5);

    // Grab rings
    leftDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct);

    // Spin to face final ring
    bot.turnTo(48, 2);

    // Pick up ring
    leftDrive.spinFor(fwd, 800, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 800, deg, 30, velocityUnits::pct);

    wait(1,sec);

    bot.turnTo(110, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.drive(reverse, 8, 20);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.drive(fwd, 7, 20);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    bot.turnTo(0,0.3);

    bot.drive(reverse, 69, 40);

    bot.drive(reverse, 9, 20);

    // Clamp
    clampPneumatic.set(true);

    bot.drive(reverse, 5, 20);

    wait(100, msec);

    // ************** SECOND CORNER **************
    
    // Spin to face first ring
    bot.turnTo(90, 1);

    // Start intake
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    // Grab ring
    bot.drive(fwd, 32, 30);

    // Face second ring
    bot.turnTo(155, 1);

    bot.drive(fwd, 40);

    bot.drive(reverse, 4);

    // Spin to face third ring
    bot.turnTo(285, 2);

    // Grab ring
    bot.drive(fwd, 23);

    // Spin to fourth and fifth rings
    bot.turnTo(270, 1);

    // Grab rings
    leftDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct, false);
    rightDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct);

    wait(0.5,sec);

    // Spin to face final ring
    bot.turnTo(135, 2);

    // Pick up ring
    bot.drive(fwd, 20);

    wait(1,sec);

    bot.turnTo(75, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.drive(reverse, 12, 22);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.drive(fwd, 5, 20);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Third Corner
    bot.turnTo(80, 1);

    // Grab and store first ring
    intakeLower.spin(fwd, 100, pct);
    bot.drive(fwd, 78, 30);
    intakeUpper.spinFor(fwd, 200, deg, false);

    // Grab and store second ring
    bot.turnTo(0, 2);
    bot.drive(fwd, 28);
    intakeUpper.spinFor(fwd, 200, deg, false);

    bot.turnTo(225, 2);
    intakeLower.stop(coast);

    // Get goal
    bot.drive(reverse, 35);
    clampPneumatic.set(true);
    bot.drive(reverse, 4);

    // Score rings
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    // Grab 2 rings
    bot.turnTo(180, 3);
    bot.drive(fwd, 72);

    // Grab last ring
    bot.turnTo(45, 3);
    bot.drive(fwd, 20);

    // Drive goal into corner
    bot.turnTo(340, 3);
    intakeLower.stop(coast);
    intakeUpper.stop(coast);
    bot.drive(reverse, 8);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);
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