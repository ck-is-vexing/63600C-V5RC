#include "game/pre-auton.h"
#include "game/autonomous.h"
#include "game/driver.h"
#include "robot-config.h"
#include "definition.h"

using namespace vex;
competition Competition;

/// Run before match begins
void pre_auton(void) {
  printl("Pre-Auton Init");

  vexcodeInit();
  driver::registerEvents();

  intakeLower.setVelocity(100, pct);
  intakeUpper.setVelocity(100, pct);
  intakeBack.setVelocity(100, pct);
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);
  redirect.setTo(true);

  preAuton::autonSelector();

  if (global::gpsAllowed == true) {
    // Wait a short period to allow GPS to register field strips
    wait(300, msec);

    // Force a GPS update before code that needs it
    Brain.Screen.print(GPS.heading()); 
  
    preAuton::inertialGPSCalibrate(1);

  } else {
    // Manual backup in case there aren't GPS strips
    imu.calibrate();
    while (imu.isCalibrating()) { wait(50, msec); }
    imu.setHeading(preAuton::inertialAngle, deg);

    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Inertial Calibrated!");
  }
}

/// Run during match autonomous
void autonomous(void) {
  printl("Auton Init");

  wing.setTo(true); // So it doesn't get stuck on things

  switch (preAuton::autonSelection) {
    case global::autonomousTypes::LEFT:
      auton::left();
      break;
    case global::autonomousTypes::RIGHT:
      auton::right();
      break;
    case global::autonomousTypes::WINPOINT:
      auton::winpoint();
      break;
    case global::autonomousTypes::TWO_INCH:
      auton::twoInch();
      break;
    case global::autonomousTypes::SKILLS:
      auton::skills();
      break;
    case global::autonomousTypes::NONE:
      break;
  }
}

/// Run during match driver control
void usercontrol(void) {
  printl("Driver Init");

  // Intake failsafe
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
  intakeBack.stop(coast);
  
  leftDrive.spin(fwd, 0, pct);
  rightDrive.spin(fwd, 0, pct);

  while (true == true /* A statement that is true */) { 

    // Manual autonomous trigger used for testing
    if (Controller1.ButtonX.pressing() && global::debugMode == true){
      //auton::PIDTest();
      wing.setTo(true); // So it doesn't get stuck on things
      
      switch (preAuton::autonSelection) {
        case global::autonomousTypes::LEFT:
          auton::left();
          break;
        case global::autonomousTypes::RIGHT:
          auton::right();
          break;
        case global::autonomousTypes::WINPOINT:
          auton::winpoint();
          break;
        case global::autonomousTypes::TWO_INCH:
          auton::twoInch();
          break;
        case global::autonomousTypes::SKILLS:
          auton::skills();
          break;
        case global::autonomousTypes::NONE:
          break;
      }

      wait(5, sec);
      leftDrive.spin(fwd, 0, pct);
      rightDrive.spin(fwd, 0, pct);
    }

    driver::checkInputs();
    wait(20, msec);
  }
}

/// Set up game calls and run pre-auton
int main() {

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  // We wouldn't want this program ending early, right? ...right?
  while (true) {
    wait(100, msec);
  }
}