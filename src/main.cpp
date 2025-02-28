// TODO: move setup functions out of main

// -------- SETUP --------

#include "robot-config.h" // Include robot configuration
#include "globals.h" // Include global variables
#include "auton/autonomous.h" // Include autonomous namespace
#include "func/button.h" // Include Button class, used for autonSelector
#include <iostream>

using namespace vex; // Set the namespace to vex

// New competition instance
competition Competition;

int autonomousNumber; // A number used to indicate which autonomous for the robot to use
int gpsBlueAngle = 90; // 90 degrees is the angle that correctly setup fields will have on the blue side
int inertialAngle = 0; // For when GPS is disabled, this number is set during the autonomous selector and applied to the inertial sensor during pre-auton


// -------- SUPPORT FUNCTIONS --------

// Check controller inputs and respond
// Tank drive with triggers controlling clamp and intake
void checkInputs() {

  // Set the drivetrain motor groups to move based on tank drive principles
  leftDrive.setVelocity(Controller1.Axis3.value(), pct);
  rightDrive.setVelocity(Controller1.Axis2.value(), pct);

  // Intake control
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

  // Wall stake mech
  if (Controller1.ButtonDown.pressing()) {
    ws.cancel();
    wallStakeMot.setVelocity(100, pct);
  } else if (Controller1.ButtonB.pressing()) {
    ws.cancel();
    wallStakeMot.setVelocity(-100, pct);
  } else if (Controller1.ButtonRight.pressing()) {
    ws.setAngle(13);
  } else {
    if (ws.running == true) {
      ws.tick();
    } else {
      wallStakeMot.setVelocity(0, pct);
    }
  }

  // Clamp control
  if (Controller1.ButtonL2.pressing()) {
    clampPneumatic.set(false);
  } else if (Controller1.ButtonL1.pressing()) {
    clampPneumatic.set(true);
  }
}

// Inertial sensor calibration function using GPS. This ends up being more precise than setting the robot up the same direction every time
// For the robot to not average the angle at all, set seconds to 0.
void inertialGPSCalibrate(double averageSeconds = 1) {

  // Setup variables
  double averagedHeading = 0; // The final averaged heading
  int i; // A counter for the for loop. This must be defined outside of it for the final division.

  // For loop adds up all the different angles of the GPS. Step one of averaging the robot angle.
  for(i = 0; i < (averageSeconds * 25) + 1; i++){

    // Add the GPS heading to the total
    averagedHeading += (GPS.heading() - gpsBlueAngle + 90);

    // Wait a short period because the GPS only updates a certain amount of times a second
    wait(40,msec);
  }

  // Divide averagedHeading by the amount of GPS headings added to it
  averagedHeading /= i;

  // Log the averaged heading versus the GPS heading
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Average: ");
  Controller1.Screen.print(averagedHeading);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("GPS: ");
  Controller1.Screen.print(GPS.heading());

  // Let the user know that the inertial sensor is now calibrating
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Calibrating...");

  // Set the inertial sensor's heading to the GPS heading.
  Inertial.setHeading(averagedHeading, deg);

  // Loop until the inertial sensor finishes setting the heading
  while(Inertial.isCalibrating()){

    //Wait to prevent wasted resources by iterating too fast
    wait(100,msec);
  }

  // Let the user know that calibration is complete
  Controller1.Screen.clearLine(3);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Inertial Calibrated!");
}

// Load interface to select autonomous program
void autonSelector() {
  
  // If debug mode is enabled, automatically set the angle and autonomous number
  /*
  if (global::debugMode == true){
    //gpsBlueAngle += 180;
    autonomousNumber = 4;
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "Skills Selected");

    // Exit the function
    return;
  }*/

  // Initialize buttons
  Button redLeft = Button(10, 10, 110, 110, "Red Left", red, white, Brain);
  Button redRight = Button(120, 10, 220, 110, "Red Right", red, white, Brain);
  Button blueLeft = Button(10, 120, 110, 220, "Blue Left", blue, white, Brain);
  Button blueRight = Button(120, 120, 220, 220, "Blue Right", blue, white, Brain);
  Button skills = Button(230, 120, 330, 220, "Skills (Red Left)", orange, white, Brain);
  Button noAuton = Button(230, 10, 330, 110, "No Auton", orange, white, Brain);
  Button decreaseAngle = Button(340, 120, 400, 180, "<", green, white, Brain);
  Button increaseAngle = Button(410, 120, 470, 180, ">", green, white, Brain);

  // Clear screen and render buttons
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

  // Safety to make sure the driver can run if an auton isn't selected
  int t = 0;
  
  // Repeatedly check for button press and notify user of what was pressed (in case of misclick)
  while (true){
    if (redLeft.isClicked() == true) {
      autonomousNumber = 0;
      inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Red Left Selected");
      break;

    } else if (redRight.isClicked() == true) {
      autonomousNumber = 1;
      inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Red Right Selected");
      break;

    } else if (blueLeft.isClicked() == true) {
      autonomousNumber = 2;
      inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Blue Left Selected");
      break;

    } else if (blueRight.isClicked() == true) {
      autonomousNumber = 3;
      inertialAngle = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Blue Right Selected");
      break;

    } else if (skills.isClicked() == true) {
      autonomousNumber = 4;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Skills Selected");
      break;

    } else if (noAuton.isClicked() == true) {
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Autonomous Cancelled");
      break;

    } else if (decreaseAngle.isClicked() == true) {
      gpsBlueAngle -= 90;
      Brain.Screen.setPenColor(white);
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(3, 39);
      Brain.Screen.print(gpsBlueAngle);
      wait(300, msec);

    } else if (increaseAngle.isClicked() == true) {
      gpsBlueAngle += 90;
      Brain.Screen.setPenColor(white);
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(3, 39);
      Brain.Screen.print(gpsBlueAngle);
      wait(300, msec);
    }

    // Quits if the loop has been running more than 30 seconds
    if (t > 1500) {
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Autonomous Aborted!");
      break;
    }

    // Increment t
    t++;

    // Wait a bit
    wait(20, msec);
  }
}


// -------- GAME FUNCTIONS --------

// Setup code ran before the competition starts
void pre_auton(void) {
  std::cout << "Pre-Auton Init" << std::endl;

  // Display the autonomous selector on the brain screen
  autonSelector();

  // Check if using the GPS is allowed
  if(global::gpsAllowed == true){
    
    // Wait a short period to allow the GPS to register field strips
    wait(300,msec);

    // Force a GPS update before the code that needs it beings
    Brain.Screen.print(GPS.heading()); 

    // Calibrate the inertial sensor with the custom calibration function over a 0.5 second averaging time
    inertialGPSCalibrate(0.5);

  } else {

    // If the robot doesn't have access to GPS strips, set the angle manually
    Inertial.setHeading(inertialAngle, deg);

    // Loop until the inertial sensor finishes calibrating.
    while(Inertial.isCalibrating()){
      wait(40,msec); // Wait to prevent wasted resources by iterating too fast
    }

    // Let the driver know that calibration is complete
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Inertial Calibrated!");

    //bot.demoTo(0);
  }

  // Set Wall stake mechanism motor to brake
  wallStakeMot.setStopping(brake);

  // Set the intake to run at 100% speed
  intakeLower.setVelocity(100, pct);
  intakeUpper.setVelocity(100, pct);

  // Make sure the clamp is not engaged
  clampPneumatic.set(false);

  // Initialize Robot Configuration
  vexcodeInit();
}

// Function run during the autonomous period
void autonomous(void) {
  std::cout << "Auton Init" << std::endl;
  
  // Temporarily set stopping to brake
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  // Doesn't run if it is skills or something else
  if (autonomousNumber < 4){

    // Schedule the intake to stop just before the end of the autonomous period
    // Rings occasionally get stuck and take longer than usual, this is just a precaution
    timer().event(stopIntake, 14800);
  }

  // Run the autonomous that was selected during the pre-auton phase
  switch (autonomousNumber) {
    case 0:
      auton::redLeft();
      break;
    case 1:
      auton::redRight();
      break;
    case 2:
      auton::blueLeft();
      break;
    case 3:
      auton::blueRight();
      break;
    case 4:
      auton::skills();
      break;
  }

  // Revert stopping to coast
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);
}

// Code run during the driver control period
void usercontrol(void) {
  std::cout << "Driver Init" << std::endl;

  // If the intake wasn't stopped previously, stop it now (just in case)
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
  
  // Prime the drivetrain motors
  leftDrive.spin(fwd, 0, pct);
  rightDrive.spin(fwd, 0, pct);

  // Prime wall stake mech
  wallStakeMot.spin(fwd, 0, pct);
  // Main loop for driver control code
  while (true == true /* A statement that is true */) { 
    
    // Manual autonomous trigger used for testing
    if (Controller1.ButtonX.pressing() && global::debugMode == true){

      //bot.driveTo(fwd, 24, 0);
      
      leftDrive.setStopping(brake);
      rightDrive.setStopping(brake);

      // Doesn't run if it is skills or something else
      if (autonomousNumber < 4){
        // Schedule the intake to stop just before the end of the autonomous period
        // Rings occasionally get stuck and take longer than usual, this is just a precaution
        timer().event(stopIntake, 14800);
      }

      // Run the autonomous that was selected during the pre-auton phase
      switch (autonomousNumber) {
        case 0:
          auton::redLeft();
          break;
        case 1:
          auton::redRight();
          break;
        case 2:
          auton::blueLeft();
          break;
        case 3:
          auton::blueRight();
          break;
        case 4:
          auton::skills();
          break;
      }

      // Start and stop motors
      leftDrive.setStopping(coast);
      rightDrive.setStopping(coast);
      leftDrive.spin(fwd);
      rightDrive.spin(fwd);
      wallStakeMot.spin(fwd); 
      
    }
    
    //renderRobot(); // Render the robot on the brain screen, useful for testing, not needed in competition
    checkInputs(); // Call checkInputs, which checks buttons and joysticks on the controller and responds accordingly
    wait(20, msec); // Sleep the program for a short amount of time to prevent wasted resources
  }
}

// Main sets up callbacks and runs the pre-auton function
int main() {

  // Set up callbacks for autonomous and driver control periods
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function
  pre_auton();

  // Prevent main from exiting with an infinite loop
  // We wouldn't want this program ending early, right? ...right?
  while (true) {
    wait(100, msec);
  }
}