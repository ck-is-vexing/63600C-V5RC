#include "robot-config.h"
#include "global.h"
#include "game/autonomous.h"
#include "game/preAuton.h"
#include "func/button.h"
#include <iostream>
#include <random>
#include <ctime>

using namespace vex;
competition Competition;

enum global::autonomousTypes autonSelection;
int inertialAngle = 0; // For when GPS is disabled

int swtch;

/// Check controller inputs and respond
void checkInputs() {

  if (Controller1.ButtonL1.pressing()) {

    if (swtch == 0) {
      leftDrive.setVelocity(Controller1.Axis3.position(), pct);
      rightDrive.setVelocity(Controller1.Axis2.position(), pct);
    } else {
      leftDrive.setVelocity(leftJoystick.getValue(), pct);
      rightDrive.setVelocity(rightJoystick.getValue(), pct);
    }

  } else if (Controller1.ButtonR1.pressing()) {

    if (swtch == 1) {
      leftDrive.setVelocity(Controller1.Axis3.position(), pct);
      rightDrive.setVelocity(Controller1.Axis2.position(), pct);
    } else {
      leftDrive.setVelocity(leftJoystick.getValue(), pct);
      rightDrive.setVelocity(rightJoystick.getValue(), pct);
    }

  } else {
    leftDrive.setVelocity(0, pct);
    rightDrive.setVelocity(0, pct);
  }

  leftDrive.setVelocity(leftJoystick.getValue(), pct);
  rightDrive.setVelocity(rightJoystick.getValue(), pct);

  if (Controller1.ButtonR1.pressing()) {
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,100,pct);
  } else if (Controller1.ButtonR2.pressing()) {
    intakeLower.spin(reverse,30,pct);
    intakeUpper.spin(reverse,30,pct);
  } else if (Controller1.ButtonA.pressing()) {
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,50,pct);
  } else {
    intakeLower.stop(coast);
    intakeUpper.stop(coast);
  }
}

/// Run before match begins
void pre_auton(void) {
  std::cout << "Pre-Auton Init" << std::endl;

  preAuton::autonSelector();

  if (global::gpsAllowed == true) {
    
    // Wait a short period to allow GPS to register field strips
    wait(300,msec);

    // Force a GPS update before code that needs it
    Brain.Screen.print(GPS.heading()); 

    preAuton::inertialGPSCalibrate(0.5);

  } else {

    // Manual backup in case there aren't GPS strips
    Inertial.calibrate();
    while (Inertial.isCalibrating()) { wait(100,msec); }
    Inertial.setHeading(inertialAngle, deg);

    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Inertial Calibrated!");
  }

  intakeLower.setVelocity(100, pct);
  intakeUpper.setVelocity(100, pct);
  intakeBack.setVelocity(100, pct);

  intakePneumatic.set(false);

  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);

  // Initialize Robot Configuration
  vexcodeInit();
}

/// Run during match autonomous
void autonomous(void) {
  std::cout << "Auton Init" << std::endl;

  switch (autonSelection) {
    case global::RED_LEFT:
      auton::redLeft();
      break;
    case global::RED_RIGHT:
      auton::redRight();
      break;
    case global::BLUE_LEFT:
      auton::blueLeft();
      break;
    case global::BLUE_RIGHT:
      auton::blueRight();
      break;
    case global::SKILLS:
      auton::skills();
      break;
    case global::NONE:
      break;
  }
}

/// Run during match driver control
void usercontrol(void) {
  std::cout << "Driver Init" << std::endl;

  std::srand(timer::system());
  swtch = std::rand() % 2;
  std::cout << swtch << std::endl;

  // Intake failsafe
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
  intakeBack.stop(coast);
  
  leftDrive.spin(fwd, 0, pct);
  rightDrive.spin(fwd, 0, pct);

  while (true == true /* A statement that is true */) { 
    
    // Manual autonomous trigger used for testing
    if (Controller1.ButtonX.pressing() && global::debugMode == true){

      switch (autonSelection) {
        case global::RED_LEFT:
          auton::redLeft();
          break;
        case global::RED_RIGHT:
          auton::redRight();
          break;
        case global::BLUE_LEFT:
          auton::blueLeft();
          break;
        case global::BLUE_RIGHT:
          auton::blueRight();
          break;
        case global::SKILLS:
          auton::skills();
          break;
        case global::NONE:
          break;
      }
      
      leftDrive.spin(fwd, 0, pct);
      rightDrive.spin(fwd, 0, pct);
    }
    
    if (Controller1.ButtonA.pressing()) {
      if (swtch == 1) {
        std::cout << "Selected Normal" << std::endl;
      } else {
        std::cout << swtch << "Selected Rapid Trigger" << std::endl;
      }
      break;

    } else if (Controller1.ButtonLeft.pressing()) {
      if (swtch == 0) {
        std::cout << "Selected Normal" << std::endl;
      } else {
        std::cout << swtch << "Selected Rapid Trigger" << std::endl;
      }
      break;
    }

    //renderRobot(); // Render the robot on the brain screen, useful for testing, not needed in competition
    checkInputs();
    wait(20, msec);
  }
}

/// Set up callbacks and run pre-auton
int main() {

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  // We wouldn't want this program ending early, right? ...right?
  while (true) {
    wait(100, msec);
  }
}