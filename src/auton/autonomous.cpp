#include "auton/autonomous.h"
#include "robot-config.h"

namespace auton {

  void blueLeft() {

    bot.drive(fwd, 5.5, 10);
    wallStakeMot.spinFor(fwd, 600, deg, 100, velocityUnits::pct);
    wallStakeMot.spinFor(reverse, 600, deg, 100, velocityUnits::pct, false);
    bot.drive(reverse, 4, 30);
    bot.turnTo(90, 2);
    bot.drive(reverse, 12);

    bot.turnTo(60, 2);

    // Drive backwards up to the goal
    bot.drive(reverse, 34, 30);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);
    
    // Drive backwards to safety
    bot.drive(fwd, 7, 40);
    
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    bot.turnTo(180, 2);

    bot.drive(fwd, 22, 50);

    wait(1000,msec);

    bot.turnTo(0, 2);

    bot.drive(fwd, 50);
  }

  void blueRight() {
    // Drive backwards up to the goal
    bot.drive(reverse, 39, 30);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);
    
    // Drive backwards
    bot.drive(fwd, 3, 40);
    
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    bot.turnTo(0, 2);

    bot.drive(fwd, 22, 40);
    intakeLower.stop(coast);
    bot.drive(fwd, 4, 40);

    bot.turnTo(270, 2);
    intakeLower.spin(fwd,100,pct);

    bot.drive(fwd, 15, 30);
    wait(400,msec);
    bot.drive(reverse, 15, 30);
    wait(400,msec);
    bot.turnTo(180, 2);

    bot.drive(fwd, 54);
  }

  void redLeft() {
    // Drive backwards up to the goal
    bot.drive(reverse, 39, 30);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);
    
    // Drive backwards
    bot.drive(fwd, 3, 40);
    
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    bot.turnTo(0, 2);

    bot.drive(fwd, 22, 40);
    intakeLower.stop(coast);
    bot.drive(fwd, 4, 40);

    bot.turnTo(90, 2);
    intakeLower.spin(fwd,100,pct);

    bot.drive(fwd, 15, 30);
    wait(400,msec);
    bot.drive(reverse, 16, 30);
    wait(400,msec);
    bot.turnTo(180, 2);

    bot.drive(fwd, 54);
  }

  void redRight() {
    // Drive backwards up to the goal
    bot.drive(reverse, 39, 30);

    // Stop the drivetrain not too hard
    leftDrive.stop(coast);
    rightDrive.stop(coast);

    // Clamp onto the goal
    clampPneumatic.set(true);

    // Wait a little bit
    wait(400, msec);
    
    // Drive backwards to safety
    bot.drive(fwd, 7, 40);
    
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    bot.turnTo(180, 2);

    bot.drive(fwd, 22, 50);

    wait(1000,msec);

    bot.turnTo(0, 2);

    bot.drive(fwd, 50);
  }

  void skillsDT() {

    wallStakeMot.spinFor(fwd, 600, deg, 100, velocityUnits::pct);
    wallStakeMot.spinFor(reverse, 600, deg, 100, velocityUnits::pct, false);
    bot.driveTo(reverse, -12, 270);
    bot.turnTo(180, 2);

    bot.driveTo(reverse, -24, 180);
    clampPneumatic.set(true);

    // ************** FIRST CORNER **************

    wait(200, msec);
    intakeLower.spin(fwd,100,pct);

    // Spin to face first ring
    bot.turnTo(90, 1.5);

    // Grab ring
    bot.driveTo(fwd, 27, 90);

    // Spin to face second ring
    bot.turnTo(45, 2);
    
    // Grab ring
    bot.driveTo(fwd, 30, 45);

    // Face wall stake
    bot.turnTo(0, 2.5);

    int t = 0;
    intakeUpper.spin(fwd, 100, pct);
    ws.setAngle(13);
    while (t < 800) {
      ws.tick();
      if (wallStakeMot.torque(Nm) >= 0.56) {
        intakeUpper.stop(coast);
        ws.setAngle(113);
      }
      if (ws.angle() >= 113) {
        ws.cancel();
        ws.stop(coast);
        break;
      }
      t++;
      wait(20, msec);
    }

    wait(100, msec);

    bot.driveTo(fwd, 12, 0);

    intakeUpper.spin(fwd, 100, pct);

    bot.driveTo(reverse, 12, 0);

    // Spin to third and fourth rings
    bot.turnTo(270, 1.5);

    // Grab rings
    bot.driveTo(fwd, 56, 270);

    // Spin to face final ring
    bot.turnTo(48, 2);

    // Pick up ring
    bot.driveTo(fwd, 13.5, 48);

    bot.turnTo(110, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.driveTo(reverse, 8, 110);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.driveTo(fwd, 7, 110);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    bot.turnTo(0, 0.3);

    bot.driveTo(reverse, 83, 0);

    // Clamp
    clampPneumatic.set(true);

    wait(100, msec);

    // ************** SECOND CORNER **************
    
    // Spin to face first ring
    bot.turnTo(90, 1);

    // Start intake
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    // Grab ring
    bot.driveTo(fwd, 32, 90);

    // Face second ring
    bot.turnTo(155, 1);

    bot.driveTo(fwd, 40, 155);

    bot.driveTo(reverse, 4, 155);

    // Spin to face third ring
    bot.turnTo(285, 2);

    // Grab ring
    bot.driveTo(fwd, 23, 285);

    // Spin to fourth and fifth rings
    bot.turnTo(270, 1);

    // Grab rings
    bot.driveTo(fwd, 34, 270);

    // Spin to face final ring
    bot.turnTo(135, 2);

    // Pick up ring
    bot.driveTo(fwd, 20, 135);

    bot.turnTo(75, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.driveTo(reverse, 12, 75);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.driveTo(fwd, 5, 75);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    
    // ************** SECOND HALF **************

    bot.turnTo(0, 2);
    bot.driveTo(fwd, 15, 0);

    // zoop under the bar!
    bot.turnTo(45, 0.8);
    leftDrive.setStopping(coast);
    rightDrive.setStopping(coast);
    intakeLower.spin(fwd,100,pct);
    bot.driveTo(fwd, 75, 45);

    // First ring
    intakeUpper.spinFor(fwd, 800, deg, false);
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);

    // Second ring
    bot.driveTo(fwd, 34, 45);
    intakeUpper.spinFor(fwd, 800, deg, false);

    // Goal
    bot.driveTo(reverse, 8, 45);
    intakeLower.stop(coast);
    bot.turnTo(315, 2);

    bot.driveTo(reverse, 38, 315);
    clampPneumatic.set(true);

    // Third ring
    bot.turnTo(225, 2);
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);
    bot.driveTo(fwd, 35, 225);

    // Spin to fourth ring
    bot.turnTo(135, 1.5);

    // Grab ring
    bot.driveTo(fwd, 31, 135);

    // Spin to face final ring
    bot.turnTo(180, 2);

    // Pick up ring
    bot.driveTo(fwd, 14, 180);

    bot.turnTo(290, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Reverse goal into corner
    bot.driveTo(reverse, 10, 290);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.driveTo(fwd, 7, 40);

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

  void skills() {
    wallStakeMot.spinFor(fwd, 600, deg, 100, velocityUnits::pct);
    wallStakeMot.spinFor(reverse, 600, deg, 100, velocityUnits::pct, false);
    bot.drive(reverse, 12.5, 30);
    bot.turnTo(180, 2);

    bot.drive(reverse, 21, 30);
    clampPneumatic.set(true);
    bot.drive(reverse, 3, 30);

    // ************** FIRST CORNER **************

    wait(500, msec);
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd, 100, pct);

    // Spin to face first ring
    bot.turnTo(90, 1.5);

    // Grab ring
    bot.drive(fwd, 25);

    bot.turnTo(38, 2);
    bot.drive(fwd, 28);

    // Face wall stake
    bot.turnTo(0, 2.5);
    bot.drive(fwd, 14, 30);

    int t = 0;
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd, 100, pct);
    ws.setAngle(13);
    while (t < 800) {
      ws.tick();
      if (wallStakeMot.torque(Nm) >= 0.54) {
        intakeUpper.stop(coast);
        ws.cancel();
        break;
      }
      t++;
      wait(20, msec);
    }
    wallStakeMot.spinFor(fwd, 1, sec, 100, velocityUnits::pct);
    wallStakeMot.spinFor(reverse, 500, deg, 100, velocityUnits::pct);

    wait(100, msec);
    intakeUpper.spin(fwd,100,pct);

    bot.drive(reverse, 12);
    ws.stop();

    // Spin to third and fourth rings
    bot.turnTo(270, 1.5);

    // Grab rings
    bot.drive(fwd, 56, 30);

    wait(200, msec);

    // Spin to face final ring
    bot.turnTo(45, 2);

    // Pick up ring
    bot.drive(fwd, 12);

    bot.turnTo(110, 3);

    wait(0.5,sec);

    // Reverse goal into corner
    bot.drive(reverse, 10, 20);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Let go
    clampPneumatic.set(false);
    intakeLower.spin(reverse,100,pct);
    intakeUpper.spin(reverse,100,pct);

    wait(400,msec);

    bot.drive(fwd, 6.5, 20);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);
    
    bot.turnTo(0, 0.3);

    bot.driveTo(reverse, -98, 0, 11);

    // Clamp
    clampPneumatic.set(true);

    wait(100, msec);

    // ************** SECOND CORNER **************
    
    // Spin to face first ring
    bot.turnTo(90, 1);

    // Start intake
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);

    // Grab ring
    bot.drive(fwd, 25, 30);

    // Face second ring
    bot.turnTo(180, 1);

    bot.drive(fwd, 23);

    // Spin to fourth and fifth rings
    bot.turnTo(270, 1);

    // Grab rings
    bot.drive(fwd, 33);

    wait(0.2,sec);

    // Spin to face final ring
    bot.turnTo(135, 2);

    // Pick up ring
    bot.drive(fwd, 20);

    wait(0.5,sec);

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
    /*bot.turnTo(0, 2);
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
    bot.drive(reverse, 10);
    intakeLower.stop(coast);
    
    bot.turnTo(315, 2);

    bot.drive(reverse, 38, 40);
    clampPneumatic.set(true);
    
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

    bot.turnTo(290, 2);

    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    // Let go
    clampPneumatic.set(false);

    // Slam into corner
    bot.drive(reverse, 10, 100);
    
    bot.drive(fwd, 99);
    bot.turnTo(315, 4);
    bot.drive(reverse, 100);
  
    bot.turnTo(340, 5);
    bot.drive(reverse, 65, 100);
    intakeLower.stop(coast);
    intakeUpper.stop(coast);

    clampPneumatic.set(false);
    wait(200, msec);

    bot.drive(fwd, 20, 100);
    */

    bot.turnTo(67, 2);
    intakeLower.spin(fwd, 100, pct);
    intakeUpper.spin(fwd, 100, pct);
    bot.drive(fwd, 105, 60);
    bot.turnTo(120, 2);
    bot.drive(fwd, 50);
    intakeLower.spin(reverse, 100, pct);
    wait(400, msec);
    bot.turnTo(165, 4);
    bot.drive(reverse, 52, 100);

    // Push final goal into corner
    bot.turnTo(15, 5);
    intakeLower.spin(fwd,30,pct);
    bot.drive(fwd, 72, 100);
    intakeLower.spin(reverse,100,pct);

    wait(200, msec);

    //bot.turnTo(45, 5);
    //bot.drive(fwd, 30, 100);

    bot.drive(reverse, 50, 100);
    intakeLower.stop(coast);
    intakeUpper.stop(coast);
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