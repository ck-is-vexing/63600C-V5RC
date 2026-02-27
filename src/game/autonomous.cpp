#include "game/autonomous.h"

#include "robot-config.h"
#include "control/intake.h"
#include "definition.h"

void auton::left() {
  bot.drive(fwd, 31, 40);

  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  wait(0.3, sec); // Required for match loader to go down

  bot.drive(fwd, 14, 40);
  wait(0.5, sec);
  bot.drive(reverse, 13, 100);

  matchLoadMech.setTo(false);
  //intakeLower.stop(coast);
  //intakeBack.stop(coast);
  //intakeUpper.stop(coast);

  bot.turnTo(177, 2);
  bot.drive(fwd, 12.5, 40);

  redirect.setTo(false);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);
  hopper.spin(fwd, 100, pct);

  wait(1.9, sec);

  intakeLower.stop(coast);
  intakeBack.stop(coast);
  intakeUpper.stop(coast);
  hopper.stop(coast);

  bot.drive(reverse, 8, 40);
  bot.turnTo(235, 2);

  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 34, 80);
  
  matchLoadMech.setTo(true);
  bot.turnTo(223, 2);

  bot.drive(fwd, 11, 40); //11

  leftDrive.stop(brake);
  rightDrive.stop(brake);

  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(reverse, 100, pct);
  hopper.spin(fwd, 100, pct);  

  wait(5, sec);
}

void auton::right() {

  // Grab 3 blocks
  bot.pointTo(pose::Pos(24, -24), 3);
  intake::store(100);
  bot.driveTo(pose::Pos(24, -24));

  wait(100, msec);

  bot.pointTo(pose::Pos(48, -48), 5);


  // First match loader
  bot.driveTo(pose::Pos(48, -48));
  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  intake::store(100);
  wait(0.3, sec); // Required for match loader to go down in time

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  wait(100, msec);
  intake::stop(coast);

  
  // Score long goal
  bot.turnTo(0);
}

void auton::winpoint() {

  // First match loader
  bot.driveTo(pose::Pos(-46, -48));
  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  intake::store(100);

  wait(0.3, sec); // Required for match loader to go down in time

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(1, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  intake::preload();
  wait(100, msec);


  // Score in first long goal
  aligner.setTo(true);
  bot.turnTo(180, 5);
  bot.driveTo(pose::Pos(-46, -30));

  intake::isPreloading = false;
  bot.drive(fwd, 20, velocityUnits::pct);

  intake::scoreLongGoal(100);
  wait(1.9, sec);
  intake::stop(coast);
  bot.stop();
  

  // Grab middle blocks
  bot.driveTo(pose::Pos(-48, -40));
  bot.turnTo(225, 5);
  aligner.setTo(false);
  
  intake::store(100);

  bot.driveTo(pose::Pos(-26, -24));
  bot.turnTo(270, 5);
  bot.driveTo(pose::Pos(21, -24));


  // Second match loader
  bot.turnTo(325, 5);
  bot.driveTo(pose::Pos(46, -48));
  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  wait(0.3, sec); // Required for match loader to go down in time

  bot.drive(fwd, 30, velocityUnits::pct);
  wait(1, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  intake::preload();
  wait(100, msec);


  // Score in second long goal
  aligner.setTo(true);
  bot.turnTo(180, 5);
  bot.driveTo(pose::Pos(46, -30));

  intake::isPreloading = false;
  bot.drive(fwd, 20, velocityUnits::pct);

  intake::scoreLongGoal(100);
  wait(1.9, sec);
  intake::stop(coast);
  bot.stop();

  // Score low middle
  // 47.4218    -45.0734    3.10229
  // 13.9648    -11.4769    2.25749*/

  wait(1000, sec); // for some reason, the robot likes to drive forwards infinitely after auton...
}

void auton::twoInch() {
  bot.drive(fwd, 2, 20);

  wait(15, sec);
}

void auton::skills() {

  // First match loader
  bot.driveTo(pose::Pos(-46, -48));
  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  intake::store(100);
  wait(0.3, sec); // Required for match loader to go down in time

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  wait(100, msec);


  // Drive to long goal
  bot.turnTo(225, 5);
  bot.driveTo(pose::Pos(-26, -24));

  bot.turnTo(180, 5);
  bot.driveTo(pose::Pos(-24, 24));

  bot.turnTo(110, 5);
  bot.driveTo(pose::Pos(-48, 40));

  
  // Score in left long goal
  redirect.setTo(false); // set early to avoid jams
  aligner.setTo(true);
  bot.turnTo(0, 5);

  intake::isPreloading = false;
  bot.drive(fwd, 8, 30);

  intake::scoreLongGoal(100);
  wait(5, sec);
  intake::stop(coast);
  bot.stop();


  // Second match loader
  bot.driveTo(pose::Pos(-48, 48));
  aligner.setTo(false);
  bot.turnTo(180, 2);

  intake::store(100);
  matchLoadMech.setTo(true);
  wait(0.3, sec); // Required for match loader to go down in time

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  intake::preload();
  wait(100, msec);


  // Clear out park zone
  bot.driveTo(pose::Pos(-48, 60));
  bot.turnTo(250, 3);
  
  odomRetract.setTo(false);
  intake::store(100);

  bot.drive(fwd, 80, 100);

  bot.turnTo(300);
  bot.drive(fwd, 15);
  bot.turnTo(270, 5);

  odomRetract.setTo(true);
  pose::odom::setPose(pose::calcPoseDist());


  // Third match loader
  bot.driveTo(pose::Pos(46, 48));
  bot.turnTo(180, 2);

  matchLoadMech.setTo(true);
  wait(0.3, sec); // Required for match loader to go down in time

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  wait(100, msec);


  // Drive to fourth match loader
  bot.turnTo(45, 10);
  intake::store(100);
  bot.driveTo(pose::Pos(26, 24));

  bot.turnTo(0, 5);
  bot.driveTo(pose::Pos(24, -24));

  bot.turnTo(315, 5);
  bot.driveTo(pose::Pos(46, -48));

  
  // Fourth match loader
  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  wait(0.3, sec); // Required for match loader to go down in time

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  intake::preload();
  wait(100, msec);


  // Score in right long goal
  aligner.setTo(true);
  redirect.setTo(false); // set early to avoid jams
  bot.turnTo(180, 5);

  intake::isPreloading = false;
  bot.drive(fwd, 24, 30);

  intake::scoreLongGoal(100);
  wait(5, sec);
  intake::stop(coast);
  bot.stop();


  // Park!
  bot.driveTo(pose::Pos(48, -42));

  bot.turnTo(90, 5);
  bot.driveTo(pose::Pos(0, -36));

  bot.turnTo(180, 4);

  odomRetract.setTo(false);
  bot.drive(reverse, 55, 100);
}

void auton::skillsMid() {

  // Score 2 in lower middle goal
  /*bot.turnTo(210, 2);
  intake::store(100);
  bot.driveTo(pose::Pos(27, -27));

  wait(100, msec);

  bot.turnTo(130, 3);
  bot.driveTo(pose::Pos(11.5, -13));

  intake::scoreLowGoal(50);
  wait(2, sec);
  intake::stop(coast);*/


  // First match loader
  bot.driveTo(pose::Pos(46, -48));
  matchLoadMech.setTo(true);
  bot.turnTo(0, 2);

  intake::store(100);

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  wait(100, msec);
  intake::stop(coast);

  
  // Drive to second loader
  bot.pointTo(pose::Pos(26, -24), 5);
  bot.driveTo(pose::Pos(26, -24));

  bot.pointTo(pose::Pos(24, 29), 5);
  bot.driveTo(pose::Pos(24, 29));

  bot.turnTo(180, 5);
  pose::odom::setPose(pose::calcPoseDist()); // distance reset
  wait(30, msec);
  intake::store(100);

  bot.pointTo(pose::Pos(48, 48), 5);
  bot.driveTo(pose::Pos(48, 48));

  
  // Second match loader
  matchLoadMech.setTo(true);
  bot.turnTo(180, 2);

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.driveTo(pose::Pos(48, 48));
  intake::stop(coast);

  matchLoadMech.setTo(false);
  aligner.setTo(true);
  redirect.setTo(false);
  
  intakeLower.spin(fwd, 10, pct); // Manual preload
  intakeBack.spin(fwd, 15, pct);  // The thread was bugging out
  intakeUpper.spin(fwd, 10, pct);
  hopper.spin(fwd, 100, pct);

  wait(100, msec);


  // Score in right long goal
  bot.turnTo(0, 3);

  intake::stop(brake);
  
  bot.drive(fwd, 40, velocityUnits::pct);
  wait(1, sec);

  intake::scoreLongGoal(100);
  wait(4.5, sec);

  bot.stop(coast);
  intake::stop(coast);
  aligner.setTo(false);


  // Clear out park zone
  /*bot.driveTo(pose::Pos(32, 60));
  intake::store(100);
  
  leftDrive.spin(fwd, 20, pct); // Manual curve to position the robot along wall
  rightDrive.spin(fwd, 30, pct);

  wait(300, msec);
  bot.stop(coast);
  
  odomRetract.setTo(false);
  intake::store(100);

  bot.drive(fwd, 60, 100);

  bot.turnTo(45, 5);
  bot.drive(fwd, 15, 100);
  bot.turnTo(90, 5);

  odomRetract.setTo(true);
  pose::odom::setPose(pose::calcPoseDist()); // distance reset
  wait(30, msec);*/
  
  /*
    // Drive to the other side
    bot.driveTo(pose::Pos(48, 40), pose::Pos(3, 3));

    bot.pointTo(pose::Pos(0, 32), 5);
    bot.driveTo(pose::Pos(0, 32));

    bot.pointTo(pose::Pos(-30, 34), 5);
    bot.driveTo(pose::Pos(-30, 34));

    bot.pointTo(pose::Pos(-48, 48), 3);
    pose::odom::setPose(pose::calcPoseDist()); // distance reset
    wait(30, msec);
  */
  bot.driveTo(pose::Pos(48, 44), pose::Pos(3, 3));

  bot.turnTo(90, 2);
  bot.drive(fwd, 72, 100);

  bot.pointTo(pose::Pos(-48, 48), 3);
  pose::odom::setPose(pose::calcPoseDist()); // distance reset
  wait(30, msec);


  // Score lower middle goal
  /*bot.turnTo(0, 5);
  intake::outtake(100);
  bot.driveTo(pose::Pos(-24, -24));

  bot.turnTo(315, 5);
  bot.driveTo(pose::Pos(-18, 18));

  intake::scoreLowGoal(50);
  wait(3, sec);

  intake::scoreLowGoal(25);
  wait(2, sec);
  intake::stop(coast);*/

  
  // Third match loader
  bot.driveTo(pose::Pos(-48, 48));
  matchLoadMech.setTo(true);
  bot.turnTo(180, 2);

  intake::store(100);

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  intake::stop(coast);
  wait(100, msec);


  // Drive to long goal
  bot.pointTo(pose::Pos(-26, 24), 5);
  intake::store(100);
  bot.driveTo(pose::Pos(-26, 24));

  bot.pointTo(pose::Pos(-24, -28), 5);
  bot.driveTo(pose::Pos(-24, -28));
  bot.turnTo(0, 5);
  pose::odom::setPose(pose::calcPoseDist()); // distance reset
  wait(30, msec);

  bot.pointTo(pose::Pos(-47, -40), 5);
  bot.driveTo(pose::Pos(-47, -40));

  
  // Score in left long goal
  intakeLower.spin(fwd, 10, pct); // Manual preload
  intakeBack.spin(fwd, 15, pct);  // The thread was bugging out
  intakeUpper.spin(fwd, 10, pct);
  hopper.spin(fwd, 100, pct);

  redirect.setTo(false); // set early to avoid jams
  aligner.setTo(true);
  bot.turnTo(180, 5);

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(1, sec);

  intake::scoreLongGoal(100);
  wait(5, sec);

  bot.stop();
  intake::stop(coast);
  aligner.setTo(false);


  // Fourth match loader
  bot.driveTo(pose::Pos(-48, -48));
  matchLoadMech.setTo(true);
  bot.turnTo(0, 2);

  intake::store(100);

  bot.drive(fwd, 40, velocityUnits::pct);
  wait(2, sec);

  bot.stop();
  bot.drive(reverse, 13, 60);

  matchLoadMech.setTo(false);
  wait(100, msec);

  
  // Park!
  bot.driveTo(pose::Pos(-48, -48));

  bot.pointTo(pose::Pos(-24, -62), 3);
  bot.driveTo(pose::Pos(-24, -62));
  odomRetract.setTo(false);

  leftDrive.spin(fwd, 40, pct); // Manual curve to position the robot along wall
  rightDrive.spin(fwd, 55, pct);

  wait(400, msec);
  bot.stop(coast);
  
  intake::store(100);

  bot.drive(reverse, 8, 100);
  bot.drive(fwd, 60, 100);

  wait(5, sec);
  intake::stop(coast);

  wait(1000, sec);
}

void auton::PIDTest() {

  // Turning
  /*bot.turnTo(180, 3);
  wait(2, sec);

  bot.turnTo(90, 3);
  wait(2, sec);

  bot.turnTo(135, 3);
  wait(2, sec);

  bot.turnTo(0, 3);
  wait(2, sec);

  bot.turnTo(180, 3);
  wait(2, sec);*/


  // Driving
  bot.pointTo(pose::Pos(24, -36));
  bot.driveTo(pose::Pos(24, -36));
  wait(2, sec);

  bot.pointTo(pose::Pos(-24, -36));
  bot.driveTo(pose::Pos(-24, -36));
  wait(2, sec);

  bot.pointTo(pose::Pos(24, -36));
  bot.driveTo(pose::Pos(24, -36));
  wait(2, sec);

  bot.driveTo(pose::Pos(-24, -36));
  wait(2, sec);
}