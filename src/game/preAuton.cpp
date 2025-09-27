#include "game/preAuton.h"

int preAuton::gpsBlueAngle = 90;
int preAuton::inertialAngle = 0;
global::autonomousTypes preAuton::autonSelection = global::autonomousTypes::NONE;

void preAuton::autonSelector() {
  
  Button redLeft = Button(10, 10, 110, 110, "Red Left", red, white, Brain);
  Button redRight = Button(120, 10, 220, 110, "Red Right", red, white, Brain);
  Button blueLeft = Button(10, 120, 110, 220, "Blue Left", blue, white, Brain);
  Button blueRight = Button(120, 120, 220, 220, "Blue Right", blue, white, Brain);
  Button skills = Button(230, 120, 330, 220, "Skills (Red Left)", orange, white, Brain);
  Button noAuton = Button(230, 10, 330, 110, "No Auton", orange, white, Brain);
  Button decreaseAngle = Button(340, 120, 400, 180, "<", green, white, Brain);
  Button increaseAngle = Button(410, 120, 470, 180, ">", green, white, Brain);

  Brain.Screen.clearScreen();
  redLeft.render();
  redRight.render();
  blueLeft.render();
  blueRight.render();
  skills.render();
  noAuton.render();
  decreaseAngle.render();
  increaseAngle.render();

  Brain.Screen.setPenColor(white);
  Brain.Screen.setCursor(2, 34);
  Brain.Screen.print("Blue Side Angle:");
  Brain.Screen.setCursor(3, 39);
  Brain.Screen.print(90);

  int t = 0;  
  while (true) {

    if (redLeft.isClicked() == true) {
      preAuton::autonSelection = global::autonomousTypes::RED_LEFT;
      preAuton::inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Red Left Selected");
      break;

    } else if (redRight.isClicked() == true) {
      preAuton::autonSelection = global::autonomousTypes::RED_RIGHT;
      preAuton::inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Red Right Selected");
      break;

    } else if (blueLeft.isClicked() == true) {
      preAuton::autonSelection = global::autonomousTypes::BLUE_LEFT;
      preAuton::inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Blue Left Selected");
      break;

    } else if (blueRight.isClicked() == true) {
      preAuton::autonSelection = global::autonomousTypes::BLUE_RIGHT;
      preAuton::inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Blue Right Selected");
      break;

    } else if (skills.isClicked() == true) {
      preAuton::autonSelection = global::autonomousTypes::SKILLS;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Skills Selected");
      break;

    } else if (noAuton.isClicked() == true) {
      preAuton::autonSelection = global::autonomousTypes::NONE;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Autonomous Cancelled");
      break;

    } else if (decreaseAngle.isClicked() == true) {
      preAuton::gpsBlueAngle -= 90;
      Brain.Screen.setPenColor(white);
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(3, 39);
      Brain.Screen.print(preAuton::gpsBlueAngle);
      wait(300, msec);

    } else if (increaseAngle.isClicked() == true) {
      preAuton::gpsBlueAngle += 90;
      Brain.Screen.setPenColor(white);
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(3, 39);
      Brain.Screen.print(preAuton::gpsBlueAngle);
      wait(300, msec);
    }

    // Quits if loop has been running more than 30 seconds
    if (t > 1500) {
      preAuton::autonSelection = global::autonomousTypes::NONE;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Autonomous Aborted!");
      break;
    }

    t++;
    wait(20, msec);
  }
}

void preAuton::inertialGPSCalibrate(double averageSeconds) {

  double averagedHeading = 0;
  int i;

  // Sum of GPS angles
  for(i = 0; i < (averageSeconds * 25) + 1; i++) {

    averagedHeading += (GPS.heading() - preAuton::gpsBlueAngle + 90);
    wait(40,msec); // Update frequency of GPS sensor
  }

  averagedHeading /= i;

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Average: ");
  Controller1.Screen.print(averagedHeading);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("GPS: ");
  Controller1.Screen.print(GPS.heading());

  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Calibrating...");

  Inertial.calibrate();
  while (Inertial.isCalibrating()) { wait(100,msec); }
  Inertial.setHeading(averagedHeading, deg);

  Controller1.Screen.clearLine(3);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Inertial Calibrated!");
}