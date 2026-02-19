#include "game/autonomous.h"
#include "robot-config.h"

void auton::left() {
  bot.drive(fwd, 31, 40);

  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  wait(0.3, sec); // Required for match loader

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
  bot.drive(fwd, 31, 40);

  bot.turnTo(0, 2);

  matchLoadMech.setTo(true);
  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  wait(0.3, sec); // Required for match loader

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
  bot.turnTo(125, 2);

  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 34, 80);
  
  bot.turnTo(135, 2);

  bot.drive(fwd, 12, 40);

  intakeLower.spin(reverse, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);
  hopper.spin(fwd, 100, pct);  
}

//TODO: Code winpoint auton
void auton::winpoint() {
  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 30.3, 30);
  bot.turnTo(135);

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
  bot.turnTo(85);

  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 48, 40);

  wait(0.5, sec);

  bot.drive(reverse, 6, 30);

  bot.turnTo(225);
  bot.drive(fwd, 15.5, 30);

  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(reverse, 100, pct);
  hopper.spin(fwd, 100, pct);
  matchLoadMech.setTo(true);

  wait(3, sec);

  intakeLower.stop(coast);
  intakeBack.stop(coast);
  intakeUpper.stop(coast);
  hopper.stop(coast);
  matchLoadMech.setTo(false);
}

void auton::twoInch() {
  bot.drive(fwd, 2, 20);

  wait(15, sec);
}

void auton::skills() {
  bot.drive(fwd, 31, 20);

  bot.turnTo(0);

  matchLoadMech.setTo(true);
  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  wait(0.5, sec); // Required for match loader

  bot.drive(fwd, 15, 40);

  leftDrive.spin(fwd, 10, pct);
  rightDrive.spin(fwd, 10, pct);
  wait(2, sec);
  leftDrive.stop(coast);
  rightDrive.stop(coast);

  bot.drive(reverse, 13.5, 20);

  matchLoadMech.setTo(false);
  //intakeLower.stop(coast);
  //intakeBack.stop(coast);
  //intakeUpper.stop(coast);

  bot.turnTo(180);
  bot.drive(fwd, 12.5, 20);

  redirect.setTo(false);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);
  hopper.spin(fwd, 100, pct);

  wait(4, sec);

  intakeLower.stop(coast);
  intakeBack.stop(coast);
  intakeUpper.stop(coast);
  hopper.stop(coast);

  bot.drive(reverse, 8, 20);
  bot.turnTo(235);

  redirect.setTo(true);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 34, 20);
  
  matchLoadMech.setTo(true);
  bot.turnTo(225);

  bot.drive(fwd, 12, 20);

  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(reverse, 30, pct);
  hopper.spin(fwd, 100, pct); 
  
  wait(5, sec);

  intakeLower.stop(coast);
  intakeBack.stop(coast);
  intakeUpper.stop(coast);
  hopper.stop(coast);

  //bot.drive(reverse, 20, 20);
  bot.drive(reverse, 17, 20);
  matchLoadMech.setTo(false);
  
  /*bot.turnTo(90);
  bot.drive(reverse, 27, 20);
  bot.turnTo(180);
  bot.drive(reverse, 60, 100);*/

  bot.turnTo(268);

  redirect.setTo(true);
  intakeLower.spin(reverse, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);

  bot.drive(fwd, 48, 50);

  bot.turnTo(315);
  intakeLower.spin(fwd, 100, pct);
  bot.drive(fwd, 37, 35);

  bot.turnTo(0);

  matchLoadMech.setTo(true);

  wait(0.5, sec); // Required for match loader
  
  bot.drive(fwd, 15, 40);
  wait(2, sec);
  bot.drive(reverse, 2, 20);

  bot.drive(fwd, 2, 20);
  wait(2, sec);

  bot.drive(reverse, 13, 20);

  matchLoadMech.setTo(false);
  //intakeLower.stop(coast);
  //intakeBack.stop(coast);
  //intakeUpper.stop(coast);

  bot.turnTo(178);
  bot.drive(fwd, 12.5, 20);

  redirect.setTo(false);
  intakeLower.spin(fwd, 100, pct);
  intakeBack.spin(fwd, 100, pct);
  intakeUpper.spin(fwd, 100, pct);
  hopper.spin(fwd, 100, pct);

  wait(4, sec);

  intakeLower.stop(coast);
  intakeBack.stop(coast);
  intakeUpper.stop(coast);
  hopper.stop(coast);

  bot.drive(reverse, 8, 20);
  bot.turnTo(90);

  bot.drive(fwd, 48, 35);
  bot.turnTo(180);

  bot.drive(reverse, 60, 100);
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