// -------- SETUP --------

#include "vex.h" // Include VEX headers
#include "func/PID.cpp" // Include PID class, used for functions like turnTo
#include "func/Button.cpp" // Include Button class, used for autonSelector
#include <cmath> // cmath is used for advanced math
#include <iostream> // iostream is used to print out to the console for debugging
using namespace vex; // Set the namespace to vex

// A global instance of competition
competition Competition;


// -------- GLOBAL VARIABLES --------

const bool gpsAllowed = true; // A toggle for whether the field has GPS strips or not
int autonomousNumber; // A number used by the autonomous selector to indicate which autonomous for the robot to use
int gpsBlueAngle = 90;

const double xOffsetGPS = 2; // In inches
const double yOffsetGPS = -5.5; // In inches

PID headingPID = PID(0.6, 0, 28, 10, true);
PID fancyDrivePID = PID(1, 0, 0, 40); // 40ms because of gps update rate
PID drivePID = PID(0.01, 0, 40, 10);

// -------- SUPPORT FUNCTIONS --------

// Returns the x position of the robot center in inches
double get_x() {

  double theta = 180 - atan(xOffsetGPS / yOffsetGPS) - (Inertial.heading() * M_PI / 180);
  double deltaX = cos(theta) * sqrt(xOffsetGPS * xOffsetGPS + yOffsetGPS * yOffsetGPS);

  return (GPS.xPosition(distanceUnits::in) + deltaX);
}

// Returns the y position of the robot center in inches
double get_y() {

  double theta = 180 - atan(xOffsetGPS / yOffsetGPS) - (Inertial.heading() * M_PI / 180);
  double deltaY = sin(theta) * sqrt(xOffsetGPS * xOffsetGPS + yOffsetGPS * yOffsetGPS);

  return (GPS.yPosition(distanceUnits::in) + deltaY);
}

// Renders an approximation of the robot position on the Brain screen
void renderRobot() {

  // VEX Brain is 480x240p
  // 20p = 1 foot
  // GPS sensor (0,0) is at the center of the field
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(white);
  Brain.Screen.setPenWidth(2);

  Brain.Screen.setFillColor(yellow);
  Brain.Screen.drawCircle(120,120,8);

  // Draw field
  for(int x = 0; x <= 6; x++){
    Brain.Screen.drawLine(x * 40, 0, x * 40, 240);
  }

  for(int y = 0; y <= 6; y++){
    Brain.Screen.drawLine(0, y * 40, 240, y * 40);
  }

  // Draw robot vector
  double angle = Inertial.heading() * M_PI / 180; // Get angle in radians
  double x = get_x() * 5/3; // Multiplied by 5/3 to convert inches to pixels
  double x_gps = GPS.xPosition(distanceUnits::in) * 5/3;
  double x2 = x + cos(angle) * 40; // 40 is the desired length of the render of the vector
  double y = get_y() * 5/3;
  double y_gps = GPS.yPosition(distanceUnits::in) * 5/3;
  double y2 = y + sin(angle) * 40;

  Brain.Screen.setPenColor(blue);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawCircle(x_gps, y_gps, 8);

  Brain.Screen.setPenColor(red);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenWidth(6);
  Brain.Screen.drawCircle(x + 120, y + 120, 8);
  Brain.Screen.drawLine(x + 120, y + 120, x2 + 120, y2 + 120);

  Brain.Screen.setCursor(4, 30);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenWidth(1);
  Brain.Screen.print("Angle: ");
  Brain.Screen.print(angle);
  Brain.Screen.setCursor(5, 30);
  Brain.Screen.print("X: ");
  Brain.Screen.print(x_gps);
  Brain.Screen.setCursor(6, 30);
  Brain.Screen.print("Y: ");
  Brain.Screen.print(y_gps);
}

// PID enabled turn function
// desiredAngle is the setpoint
void turnTo(double desiredAngle, double precision = 0.5, double secondsAllowed = 2, int recursions = 5, double minimumSpeed = 1) {

  // Reset the PID
  headingPID.reset();

  // vars
  double currentAngle;

  // Loop of recursions
  for (int i = 0; i < recursions; i++){

    // Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){

      // Update angle
      currentAngle = Inertial.heading();
      double change = headingPID.update(desiredAngle, currentAngle);

      if (std::abs(change) < minimumSpeed) {
        change = std::copysign(minimumSpeed, change);
      }

      // Spin drivetrain
      leftDrive.spin(fwd,change,pct);
      rightDrive.spin(reverse,change,pct);

      // Check for completion of the tick loop
      if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
        (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) || //Check for completion at a near 0 or 360 degree angle
        (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){ //Same as above

        // Break the tick loop
        break;
      }

      // Prevent wasted resources by waiting a short period of time before iterating
      wait(10,msec); 
    }

    leftDrive.stop(brake);
    rightDrive.stop(brake);

    // Wait a little bit in case the system is still in motion
    wait(300,msec);

    // Update currentAngle
    currentAngle = Inertial.heading();

    // Check if desired angle was achieved
    if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
        (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) || //Check for completion at a near 0 or 360 degree angle
        (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){ //Same as above
    
      // Update controller screen to tell user the robot has finished the turn, along with debug information
      //Controller1.Screen.clearScreen();
      //Controller1.Screen.setCursor(1, 1);
      //Controller1.Screen.print("Turn Complete!");
      //Controller1.Screen.setCursor(2, 1);
      //Controller1.Screen.print("Angle: ");
      //Controller1.Screen.print(currentAngle);

      // Stop drivetrain
      leftDrive.stop(coast);
      rightDrive.stop(coast);

      // Break the recursion loop and the whole turnTo, because it has reached the setpoint
      break;
    }  

    // If the correct angle was not achieved, the code will recurse
  }
}

// PID enabled drive function using the location of the robot
// desiredX and desiredY are coordinates on the VEX Field, with (0,0) being the red negative corner, and the unit being inches.
// Does not work right now silly! 
void posDriveTo(double desiredX, double desiredY, double precision = 0.5, double secondsAllowed = 10, int recursions = 5) {

  // Calculate the angle
  // pv = sqrt((y2 - y1)^2 + (x2 - x1)^2)
  // theta = atan((y2 - y1) / (x2 - x1))
  // Turn to face the desired point
  turnTo(atan(3));

  headingPID.reset();
  fancyDrivePID.reset();

  // vars
  double currentAngle;
  double currentDistance;

  // Loop of recursions
  for (int i = 0; i < recursions; i++){

    // Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){
      

      // Update vars
      currentAngle = Inertial.heading();
      double change = headingPID.update(desiredX, currentAngle);

      // Spin drivetrain
      leftDrive.spin(fwd,change,pct);
      rightDrive.spin(reverse,change,pct);

      // Check for completion of the tick loop
      if (currentAngle < (desiredX + precision) && currentAngle > (desiredX - precision)){

        // Break the tick loop
        break;
      }

      // Prevent wasted resources by waiting a short period of time before iterating
      wait(10,msec); 
    }

    leftDrive.stop(brake);
    rightDrive.stop(brake);

    // Wait a little bit in case the system is still in motion
    wait(200,msec);

    // Update currentAngle
    currentAngle = Inertial.heading();

    // Check if desired angle was achieved
    if (currentAngle < (desiredX + precision) && currentAngle > (desiredX - precision)){
      
      // Update controller screen to tell user the robot has finished the turn, along with debug information
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Turn Complete!");
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Angle: ");
      Controller1.Screen.print(currentAngle);

      // Stop drivetrain
      leftDrive.stop(coast);
      rightDrive.stop(coast);

      // Break the recursion loop and the whole turnTo, because it has reached the setpoint
      break;
    }  

    // If the correct angle was not achieved, the code will recurse
  }
}

// PID enabled drive function without GPS
// Takes an angle and a distance
// Has issues due to motor angle data not being precise enough--should be switched to tracking wheels
void driveTo(double desiredInches, directionType direction, double desiredAngle, double precision = 0.5, double secondsAllowed = 10, int recursions = 5) {

  // Reset the PIDs
  headingPID.reset();
  drivePID.reset();

  // Reset the drivetrain positions
  leftDrive.setPosition(0, deg);
  rightDrive.setPosition(0, deg);

  // variables
  double currentDistance = 0;
  double currentAngle;
  double oldDeg = (leftDrive.position(degrees) + rightDrive.position(degrees)) / 2; // Averages the position of the two sides, in order to cancel out drift
  double currentDeg;


  // Loop of recursions
  for (int i = 0; i < recursions; i++){

    // Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){

      // Update angle
      currentAngle = Inertial.heading();
      double change = headingPID.update(desiredAngle, currentAngle);

      // Update distance
      // Distance is achieved without using the GPS by taking the integral of the derivative of motor position and converting to inches
      // Future optimizations include removing calculating the derivative by just taking the motor.velocity() directly
      currentDeg = (leftDrive.position(degrees) + rightDrive.position(degrees)) / 2;
      currentDistance += (currentDeg - oldDeg) * (0.0170169602069);
      oldDeg = currentDeg;
      std::cout << currentDistance << std::endl;

      change += drivePID.update(desiredInches, currentDistance);

      // Spin drivetrain
      leftDrive.spin(direction,change,pct);
      rightDrive.spin(direction,change,pct);

      // Check for completion of the tick loop
      if (currentDistance < (desiredInches + precision) && currentDistance > (desiredInches - precision)){ //Same as above

        // Break the tick loop
        break;
      }

      // Prevent wasted resources by waiting a short period of time before iterating
      wait(10,msec); 
    }

    leftDrive.stop(brake);
    rightDrive.stop(brake);

    // Wait a little bit in case the system is still in motion
    wait(300,msec);

    // Update currentAngle
    currentAngle = Inertial.heading();

    // Check if desired angle was achieved
    if (currentDistance < (desiredInches + precision) && currentDistance > (desiredInches - precision)){ //Same as above
    
      // Stop drivetrain using coast
      leftDrive.stop(coast);
      rightDrive.stop(coast);

      // Break the recursion loop and the whole turnTo, because it has reached the setpoint
      break;
    }  

    // If the correct angle was not achieved, the code will recurse
  }
}

// Not-so-pid drive. Takes a VEX direction and the distance in inches. Optionally add a specific velocity
void drive(directionType direction, double inches, int velocityPercent = 30){

  //Conversion from inches to degrees of rotation for the motor.
  double motorDegrees = inches / (0.0170169602069); // inches divided by (Circumference of the wheels) * (The gear ratio of the robot) / (360, to put the number into degrees)
  leftDrive.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct, false);
  rightDrive.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct);
}

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
  } else if (Controller1.ButtonA.pressing()) {
    intakeLower.spin(reverse,30,pct);
    intakeUpper.spin(reverse,30,pct);
  } else if (Controller1.ButtonR2.pressing()) {
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,50,pct);
  } else {
    intakeLower.stop(coast);
    intakeUpper.stop(coast);
  }

  // Wall stake mech
  if (Controller1.ButtonDown.pressing()) {
    wallStake.setVelocity(100, pct);
  } else if (Controller1.ButtonB.pressing()) {
    wallStake.setVelocity(-100, pct);
  } else {
    wallStake.setVelocity(0, pct);
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
  // Initialize buttons
  Button redLeft = Button(10, 10, 110, 110, "Red Left", red, white);
  Button redRight = Button(120, 10, 220, 110, "Red Right", red, white);
  Button blueLeft = Button(10, 120, 110, 220, "Blue Left", blue, white);
  Button blueRight = Button(120, 120, 220, 220, "Blue Right", blue, white);
  Button skills = Button(230, 120, 330, 220, "Skills (Red Left)", orange, white);
  Button noAuton = Button(230, 10, 330, 110, "No Auton", orange, white);
  Button decreaseAngle = Button(340, 120, 400, 180, "<", green, white);
  Button increaseAngle = Button(410, 120, 470, 180, ">", green, white);

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
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Red Left Selected");
      break;
    } else if (redRight.isClicked() == true) {
      autonomousNumber = 1;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Red Right Selected");
      break;
    } else if (blueLeft.isClicked() == true) {
      autonomousNumber = 2;
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "Blue Left Selected");
      break;
    } else if (blueRight.isClicked() == true) {
      autonomousNumber = 3;
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

// -------- AUTONOMOUS FUNCTIONS --------

// Callback to stop intake
void stopIntake(void *arg) {
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
}

// Test a couple different turnTo PID angles, used for tuning and debugging
void PIDTest() {
  // Test different angles
  turnTo(180);
  wait(2,sec);

  turnTo(90);
  wait(2,sec);

  turnTo(135);
  wait(2,sec);

  turnTo(0);
  wait(2,sec);

  turnTo(180);
  wait(2,sec);
}

// Autonomous function ran at the start of a competition match
void blueLeftGameAuton() {
  
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

// Autonomous function ran at the start of a competition match
void blueRightGameAuton() {
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
  
  turnTo(0,2);

  intakeLower.spin(fwd,100,pct);
  intakeUpper.spin(fwd,50,pct);

  wait(1,sec);

  leftDrive.spinFor(fwd, 1500, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 1500, deg, 50, velocityUnits::pct);

  wait(1,sec);

  turnTo(270,1);
  leftDrive.spinFor(fwd, 850, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 850, deg, 50, velocityUnits::pct);
  
  wait(3,sec);

}

// Autonomous function ran at the start of a competition match
void redLeftGameAuton() {
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
  
  turnTo(0);

  // Score?!!!!
  intakeUpper.spinFor(3,sec, 40, velocityUnits::pct);
  
  intakeLower.spin(fwd,100,pct);
  intakeUpper.spin(fwd,50,pct);

  leftDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct);

  wait(3000,msec);

  turnTo(180);

  leftDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct);
}

// Autonomous function ran at the start of a competition match
void redRightGameAuton() {
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
  
  turnTo(180);

  // Score?!!!!
  intakeUpper.spinFor(3,sec, 40, velocityUnits::pct);
  
  intakeLower.spin(fwd,100,pct);
  intakeUpper.spin(fwd,50,pct);

  leftDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 1300, deg, 50, velocityUnits::pct);

  wait(3000,msec);

  turnTo(0);

  leftDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 2200, deg, 50, velocityUnits::pct);
}

// Autonomous skills that doesn't use the wall stake mechanism
// SETUP is red left, facing 45 deg at goal, robot is on the side of the alliance stake
void autonSkillsAutonNWS(){

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
  turnTo(90, 1.5);

  // Grab ring
  leftDrive.spinFor(fwd, 1600, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 1600, deg, 50, velocityUnits::pct);

  // Spin to face second ring
  turnTo(0, 2);

  // Grab ring
  leftDrive.spinFor(fwd, 1550, deg, 50, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 1550, deg, 50, velocityUnits::pct);

  // Spin to third and fourth rings
  turnTo(270, 1.5);

  // Grab rings
  leftDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct);

  // Spin to face final ring
  turnTo(48, 2);

  // Pick up ring
  leftDrive.spinFor(fwd, 800, deg, 30, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 800, deg, 30, velocityUnits::pct);

  wait(1,sec);

  turnTo(110, 2);

  intakeLower.stop(coast);
  intakeUpper.stop(coast);

  // Reverse goal into corner
  drive(reverse, 8, 20);

  // Let go
  clampPneumatic.set(false);
  intakeLower.spin(reverse,100,pct);
  intakeUpper.spin(reverse,100,pct);

  wait(400,msec);

  drive(fwd, 7, 20);

  intakeLower.stop(coast);
  intakeUpper.stop(coast);

  turnTo(0,0.3);

  drive(reverse, 69, 40);

  drive(reverse, 9, 20);

  // Clamp
  clampPneumatic.set(true);

  drive(reverse, 5, 20);

  wait(100, msec);

  // ************** SECOND CORNER **************
  
  // Spin to face first ring
  turnTo(90, 1);

  // Start intake
  intakeLower.spin(fwd,100,pct);
  intakeUpper.spin(fwd,100,pct);

  // Grab ring
  drive(fwd, 32, 30);

  // Face second ring
  turnTo(155, 1);

  drive(fwd, 40);

  drive(reverse, 4);

  // Spin to face third ring
  turnTo(285, 2);

  // Grab ring
  drive(fwd, 23);

  // Spin to fourth and fifth rings
  turnTo(270, 1);

  // Grab rings
  leftDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 2000, deg, 30, velocityUnits::pct);

  wait(0.5,sec);

  // Spin to face final ring
  turnTo(135, 2);

  // Pick up ring
  drive(fwd, 20);

  wait(1,sec);

  turnTo(75, 2);

  intakeLower.stop(coast);
  intakeUpper.stop(coast);

  // Reverse goal into corner
  drive(reverse, 12, 22);

  // Let go
  clampPneumatic.set(false);
  intakeLower.spin(reverse,100,pct);
  intakeUpper.spin(reverse,100,pct);

  wait(400,msec);

  drive(fwd, 5, 20);

  intakeLower.stop(coast);
  intakeUpper.stop(coast);

  // Third Corner
  turnTo(80, 1);

  // Grab and store first ring
  intakeLower.spin(fwd, 100, pct);
  drive(fwd, 78, 30);
  intakeUpper.spinFor(fwd, 200, deg, false);

  // Grab and store second ring
  turnTo(0, 2);
  drive(fwd, 28);
  intakeUpper.spinFor(fwd, 200, deg, false);

  turnTo(225, 2);
  intakeLower.stop(coast);

  // Get goal
  drive(reverse, 35);
  clampPneumatic.set(true);
  drive(reverse, 4);

  // Score rings
  intakeLower.spin(fwd,100,pct);
  intakeUpper.spin(fwd,100,pct);

  // Grab 2 rings
  turnTo(180, 3);
  drive(fwd, 72);

  // Grab last ring
  turnTo(45, 3);
  drive(fwd, 20);

  // Drive goal into corner
  turnTo(340, 3);
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
  drive(reverse, 8);

  // Let go
  clampPneumatic.set(false);
  intakeLower.spin(reverse,100,pct);
  intakeUpper.spin(reverse,100,pct);
}

// Autonomous skills with the wall stake (yippee!)
// As of this code printout, has not been completed
void autonSkillsAuton(){

  // zoop under the bar!
  turnTo(45, 1);
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);
  intakeLower.spin(fwd,100,pct);
  drive(fwd, 75);

  // First ring
  intakeUpper.spinFor(fwd, 800, deg, false);
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  // Second ring
  drive(fwd, 34);
  intakeUpper.spinFor(fwd, 800, deg, false);

  // Goal
  drive(reverse, 8);
  intakeLower.stop(coast);
  turnTo(315, 2);

  drive(reverse, 36);
  clampPneumatic.set(true);
  drive(reverse, 2);

  // Third ring
  turnTo(225, 2);
  intakeLower.spin(fwd,100,pct);
  intakeUpper.spin(fwd,100,pct);
  drive(fwd, 35);

  // Spin to face fourth ring
  turnTo(180, 2);

  // Grab ring
  drive(fwd, 24);

  // Spin to fifth ring
  turnTo(90, 1.5);

  // Grab ring
  drive(fwd, 24);

  // Spin to face final ring
  turnTo(180, 2);

  // Pick up ring
  drive(fwd, 14);

  wait(1,sec);

  turnTo(290, 2);

  intakeLower.stop(coast);
  intakeUpper.stop(coast);

  // Reverse goal into corner
  drive(reverse, 10, 20);

  // Let go
  clampPneumatic.set(false);
  intakeLower.spin(reverse,100,pct);
  intakeUpper.spin(reverse,100,pct);

  wait(400,msec);

  drive(fwd, 7, 40);

  intakeLower.stop(coast);
  intakeUpper.stop(coast);

  // Slam into corner, just to be safe
  drive(reverse, 10, 100);
  drive(fwd, 7, 40);

  // Push final goal into corner
  turnTo(15,2);
  intakeLower.spin(fwd,30,pct);
  drive(fwd, 144, 65);

  turnTo(45, 5);
  drive(fwd, 30, 100);

  drive(reverse, 20, 100);
}

// -------- GAME FUNCTIONS --------

// Setup code ran before the competition starts
void pre_auton(void) {
  
  // Display the autonomous selector on the brain screen
  autonSelector();

  // Check if using the GPS is allowed
  if(gpsAllowed == true){
    
    // Wait a short period to allow the GPS to register field strips
    wait(300,msec);

    // Force a GPS update before the code that needs it beings
    Brain.Screen.print(GPS.heading()); 

    // Calibrate the inertial sensor with the custom calibration function over a 0.5 second averaging time
    inertialGPSCalibrate(0.5);
  
  } else {

    // If the robot doesn't have access to GPS strips, assume that it was set up facing 0 degrees
    // This needs to be improved, possibly using a heading set during the autonSelector so that it is consistent
    Inertial.setHeading(0, deg);

    // Loop until the inertial sensor finishes calibrating.
    while(Inertial.isCalibrating()){

      // Wait to prevent wasted resources by iterating too fast
      wait(40,msec);
    }

    // Let the driver know that calibration is complete
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Inertial Calibrated!");
  }

  // Set Wall stake mech to brake
  wallStake.setStopping(brake);

  // Set the intake to run at 100%
  intakeLower.setVelocity(100, pct);
  intakeUpper.setVelocity(100, pct);

  // Make sure the clamp js not engaged
  clampPneumatic.set(false);

  // Initialize Robot Configuration
  vexcodeInit();
}

// Function run during the autonomous period
void autonomous(void) {

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
  if (autonomousNumber == 0){
    redLeftGameAuton();
  } else if (autonomousNumber == 1){
    redRightGameAuton();
  } else if (autonomousNumber == 2){
    blueLeftGameAuton();
  } else if (autonomousNumber == 3){
    blueRightGameAuton();
  } else if (autonomousNumber == 4){
    autonSkillsAuton();
  }

  // Revert stopping to coast
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);
}

// Code run during the driver control period
void usercontrol(void) {

  // If the intake wasn't stopped previously, stop it now (just in case)
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
  
  // Prime the drivetrain motors
  leftDrive.spin(fwd);
  rightDrive.spin(fwd);

  // Prime wall stake mech
  wallStake.spin(fwd);

  // Main loop for driver control code
  while (true == true /*a statement that is true*/) { 

    // Manual autonomous trigger used for testing, should be commented out for competition
    /*
    if (Controller1.ButtonX.pressing()){

      leftDrive.setStopping(brake);
      rightDrive.setStopping(brake);

      // Doesn't run if it is skills or something else
      if (autonomousNumber < 4){
        // Schedule the intake to stop just before the end of the autonomous period
        // Rings occasionally get stuck and take longer than usual, this is just a precaution
        timer().event(stopIntake, 14800);
      }

      // Run the autonomous that was selected during the pre-auton phase
      if (autonomousNumber == 0){
        redLeftGameAuton();
      } else if (autonomousNumber == 1){
        redRightGameAuton(); 
      } else if (autonomousNumber == 2){
        blueLeftGameAuton();
      } else if (autonomousNumber == 3){
        blueRightGameAuton();
      } else if (autonomousNumber == 4){
        autonSkillsAuton();
      }

      // Start and stop motors
      leftDrive.setStopping(coast);
      rightDrive.setStopping(coast);
      leftDrive.spin(fwd);
      rightDrive.spin(fwd);
      wallStake.spin(fwd); 
    }
    */
    
    //renderRobot(); // Render the robot on the brain screen, useful for debugging, not needed in competition
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