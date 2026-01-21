#include "game/pre-auton.h"
#include "vex.h"
#include "robot-config.h"
#include "func/button.h"
#include "definition.h"

int preAuton::sideAngle = 180;
int preAuton::inertialAngle = 0;
autonomousTypes preAuton::autonSelection = autonomousTypes::NONE;


void preAuton::autonSelector() {
  constexpr unsigned int MAX_SECONDS = 30;

  // Select the team color -----------------------------------------------

  Button red  = Button(20,  20, 230, 220, "Red",  vex::red,  vex::white, Brain);
  Button blue = Button(250, 20, 460, 220, "Blue", vex::blue, vex::white, Brain);

  Brain.Screen.clearScreen();
  red.render();
  blue.render();

  unsigned int t = 0;
  while (true) {

    if        (red.isClicked()  == true) {
      global::yourColor = colorType::RED;
      break;

    } else if (blue.isClicked() == true) {
      global::yourColor = colorType::BLUE;
      break;
    }
    
    if (t > (MAX_SECONDS * 50)) {
      global::yourColor = colorType::NONE;
      break;
    }

    t++;
    wait(20, msec);
  }

  Brain.Screen.clearScreen();
  wait(0.5, sec);
  // Select the autonomous -----------------------------------------------

  vex::color buttonColor;

  if        (global::yourColor == colorType::RED ) {
    buttonColor = vex::red;
  } else if (global::yourColor == colorType::BLUE) {
    buttonColor = vex::blue;
  } else {
    buttonColor = vex::purple;
  }

  Button left          = Button(10,  10,  110, 110, "Left",          buttonColor, vex::white, Brain);
  Button right         = Button(120, 10,  220, 110, "Right",         buttonColor, vex::white, Brain);
  Button winpoint      = Button(10,  120, 110, 220, "Winpoint", buttonColor, vex::white, Brain);
  Button twoInch       = Button(120, 120, 220, 220, "Two Inch",      buttonColor, vex::white, Brain);
  Button skills        = Button(230, 120, 330, 220, "Skills",        vex::orange, vex::white, Brain);
  Button noAuton       = Button(230, 10,  330, 110, "No Auton",      vex::orange, vex::white, Brain);
  Button decreaseAngle = Button(340, 120, 400, 180, "<",             vex::green,  vex::white, Brain);
  Button increaseAngle = Button(410, 120, 470, 180, ">",             vex::green,  vex::white, Brain);

  Brain.Screen.clearScreen();
  left.render();
  right.render();
  winpoint.render();
  twoInch.render();
  skills.render();
  noAuton.render();
  decreaseAngle.render();
  increaseAngle.render();

  Brain.Screen.setPenColor(white);
  Brain.Screen.setCursor(2, 34);
  Brain.Screen.print("Your Side Angle:");
  Brain.Screen.setCursor(3, 39);
  Brain.Screen.print(preAuton::sideAngle);

  t = 0;  
  while (true) {

    if        (left.isClicked() == true) {
      preAuton::autonSelection = autonomousTypes::LEFT;
      preAuton::inertialAngle  = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Left Selected");
      break;

    } else if (right.isClicked() == true) {
      preAuton::autonSelection = autonomousTypes::RIGHT;
      preAuton::inertialAngle  = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Right Selected");
      break;

    } else if (winpoint.isClicked() == true) {
      preAuton::autonSelection = autonomousTypes::WINPOINT;
      preAuton::inertialAngle  = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Winpoint Selected");
      break;

    } else if (twoInch.isClicked() == true) {
      preAuton::autonSelection = autonomousTypes::TWO_INCH;
      preAuton::inertialAngle  = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Two Inch Selected");
      break;

    } else if (skills.isClicked() == true) {
      preAuton::autonSelection = autonomousTypes::SKILLS;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Skills Selected");
      break;

    } else if (noAuton.isClicked() == true) {
      preAuton::autonSelection = autonomousTypes::NONE;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Autonomous Cancelled");
      break;

    } else if (decreaseAngle.isClicked() == true) {
      preAuton::sideAngle -= 90;
      Brain.Screen.clearLine(3);
      left.render();
      right.render();
      noAuton.render();
      Brain.Screen.setPenColor(white);
      Brain.Screen.setCursor(3, 39);
      Brain.Screen.print(preAuton::sideAngle);
      wait(300, msec);

    } else if (increaseAngle.isClicked() == true) {
      preAuton::sideAngle += 90;
      Brain.Screen.clearLine(3);
      left.render();
      right.render();
      noAuton.render();
      Brain.Screen.setPenColor(white);
      Brain.Screen.setCursor(3, 39);
      Brain.Screen.print(preAuton::sideAngle);
      wait(300, msec);
    }
    
    if (t > (MAX_SECONDS * 50)) {
      preAuton::autonSelection = autonomousTypes::NONE;
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

  printl("GPS Quality: " << GPS.quality());

  // Sum of GPS angles
  for(i = 0; i < (averageSeconds * 25) + 1; i++) {

    averagedHeading += (GPS.heading() - preAuton::sideAngle);
    wait(40, msec); // Update frequency of GPS sensor
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

  imu.calibrate();
  while (imu.isCalibrating()) { wait(100, msec); }
  imu.setHeading(averagedHeading, deg);

  Controller1.Screen.clearLine(3);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Inertial Calibrated!");
}