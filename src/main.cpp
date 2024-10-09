/*
To-do:
 - Make the GPS position relative to the robot, not the GPS sensor
 - drive function needs work

*/


// -------- SETUP --------

#include "vex.h" // Include the vex header
#include "func/PID.cpp" // Include the PID class
using namespace vex; // Set the namespace to vex

// A global instance of competition
competition Competition;


// -------- GLOBAL VARIABLES --------

bool skillsMode = false; // A toggle for whether to run the game auton or skills auton
bool gpsAllowed = true; // A toggle for whether the field has GPS strips or not
bool redSide = false; // A toggle to tell where the robot is starting on the field

double startX = 48; // In inches
double startY = 5 * 24; // In inches
double xOffsetGPS = 5.25; // In inches
double yOffsetGPS = -4.5; // In inches
double angleOffsetGPS = 180; // In degrees

PID headingPID = PID(0.7, 0, 0, 10);
PID drivePID = PID(1,0,0,40); // 40 bc of gps update rate

// -------- SUPPORT FUNCTIONS --------

// Returns the x position of the robot in feet
double get_x(){
  return (GPS.xPosition(distanceUnits::in) + xOffsetGPS)/ 12;
}

// Returns the y position of the robot in feet
double get_y(){
  return (GPS.yPosition(distanceUnits::in) + yOffsetGPS)/ 12;
}

// Renders an approximation of the robot position on the Brain screen
void renderRobot(){

  // VEX Brain is 480x240p
  // 20p = 1 foot
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(white);
  Brain.Screen.setPenWidth(2);

  // Draw field
  for(int x = 0; x <= 6; x++){
    Brain.Screen.drawLine(x * 40, 0, x * 40, 240);
  }

  for(int y = 0; y <= 6; y++){
    Brain.Screen.drawLine(0, y * 40, 240, y * 40);
  }

  // Draw robot vector
  double angle = Inertial.heading(deg);
  double x = get_x() * 20; // Multiplied by 20 to convert feet to pixels
  double x2 = x + cos(angle / 40) * 40; // 40 is the desired length of the render of the vector
  double y = get_y() * -20 + 240; // Inverted because the Brain screen origin is the upper left, but we want the lower left
  double y2 = y + sin(angle / 40) * 40;

  Brain.Screen.setPenColor(red);
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenWidth(6);
  Brain.Screen.drawCircle(x, y, 8);
  Brain.Screen.drawLine(x, y, x2, y2);

  Brain.Screen.setCursor(4, 30);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenWidth(1);
  Brain.Screen.print(angle);
}
// PID enabled turn function
// desiredAngle is the setpoint
void turnTo(double desiredAngle, double precision = 0.5, double secondsAllowed = 2, int recursions = 5){

  // Reset the PID
  headingPID.reset();

  // vars
  double currentAngle;

  // Loop of recursions
  for (int i = 0; i < recursions; i++){

    // Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){
      
      /*
      //Find the shortest possible route to reach target angle
      if (error > 180){
        error -= 360;
      } else if (error < -180){
        error += 360;
      }
      */

      // Update vars
      currentAngle = Inertial.heading(deg);
      double change = headingPID.update(desiredAngle, currentAngle);

      // Spin drivetrain
      leftDrive.spin(fwd,change,pct);
      rightDrive.spin(reverse,change,pct);

      // Check for completion of the tick loop
      if (currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)){

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
    currentAngle = Inertial.heading(deg);

    // Check if desired angle was achieved
    if (currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)){
      
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

// PID enabled drive function
// desiredX and desiredY are coordinates on the VEX Field, with (0,0) being the red negative corner, and the unit being feet.
void drive(double desiredX, double desiredY, double precision = 0.5, double secondsAllowed = 2, int recursions = 5){

  // Calculate the angle
  // pv = sqrt((y2 - y1)^2 + (x2 - x1)^2)
  // theta = atan((y2 - y1) / (x2 - x1))
  // Turn to face the desired point
  turnTo(atan(3));

  headingPID.reset();
  drivePID.reset();

  // vars
  double currentAngle;
  double currentDistance;

  // Loop of recursions
  for (int i = 0; i < recursions; i++){

    // Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){
      
      /*
      //Find the shortest possible route to reach target angle
      if (error > 180){
        error -= 360;
      } else if (error < -180){
        error += 360;
      }
      */

      // Update vars
      currentAngle = Inertial.heading(deg);
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
    currentAngle = Inertial.heading(deg);

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

// Check controller inputs and respond
// Tank drive with triggers controlling clamp and intake
void checkInputs(){

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
  } else if (Controller1.ButtonB.pressing()) {
    intakeLower.spin(fwd,100,pct);
    intakeUpper.spin(fwd,50,pct);
  } else {
    intakeLower.stop(coast);
    intakeUpper.stop(coast);
  }

  // Clamp control
  if (Controller1.ButtonL2.pressing()) {
    clampPneumatic.set(false);
  } else if (Controller1.ButtonL1.pressing()) {
    clampPneumatic.set(true);
  }
}

// Inertial sensor calibration function using GPS. This ends up being more precise than setting the robot up the same direction every time.
void inertialGPSCalibrate(double averageSeconds = 1){

  // For the robot to not average the turn at all, set seconds to 0.

  // Setup variables
  double averagedHeading = 0; // The final averaged heading
  int i; // A counter for the for loop. This must be defined outside of it for the final division.

  // For loop adds up all the different angles of the GPS. Step one of averaging the robot angle.
  for(i = 0; i < (averageSeconds * 25) + 1; i++){

    // Add the GPS heading to the total
    averagedHeading += GPS.heading();

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


// -------- AUTONOMOUS FUNCTIONS --------

// Autonomous function ran at the start of a competition match when the robot is on the far field side.
void redGameAuton(){
  // The season hasn't started yet!
}

// Autonomous function ran at the start of a competition match when the robot is on the close field side
void blueGameAuton(){
  // Same as in farGameAuton!
}

// Autonomous skills run.
void autonSkillsAuton(){
  // Yet another instance of... you guessed it... the season hasn't started yet!
}

// Test a couple different PID angles
void PIDTest(){

  // Wait to make sure that calibration is complete.
  wait(5,sec);

  // Test different angles
  turnTo(180);
  wait(3,sec);

  turnTo(90);
  wait(3,sec);

  turnTo(135);
  wait(3,sec);

  turnTo(0);
  wait(3,sec);
}


// -------- GAME FUNCTIONS --------

// Setup code ran before the competition starts
void pre_auton(void) {
  
  // Check if using the GPS is allowed
  if(gpsAllowed == true){

    // Wait a short period to allow the GPS to register field strips
    wait(200,msec);

    // Force a GPS update before the code that needs it beings
    Brain.Screen.print(GPS.heading()); 
    
    // Wait a short period to allow the GPS to update
    wait(200,msec);

    // Set the GPS origin
    //GPS.setOrigin(xOffsetGPS, yOffsetGPS, inches);

    // Calibrate the inertial sensor with the custom calibration function with a 0.5 second averaging time
    inertialGPSCalibrate(0.5);
  
  } else {

    // If the robot doesn't have access to GPS strips, assume that it was set up facing 0 degrees
    Inertial.setHeading(0, deg);

    // Loop until the inertial sensor finishes calibrating.
    while(Inertial.isCalibrating()){

      // Wait to prevent wasted resources by iterating too fast
      wait(100,msec);

    }

    // Let the driver know that calibration is complete
    Controller1.Screen.setCursor(3, 1); //Set the cursor to the next available row
    Controller1.Screen.print("Inertial Calibrated!"); //Print a nice message for the driver
  }
  
  // Set drivetrain motors to coast, based on Zach's driver prefrence
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);

  // Set the clamp to be correct;
  clampPneumatic.set(false);

  // Initialize Robot Configuration
  vexcodeInit();
}

// Function run during the autonomous period
void autonomous(void) {
  
  // Drive backwards up to the goal
  leftDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct, false);
  rightDrive.spinFor(reverse, 2300, deg, 30, velocityUnits::pct);
  /*
  // Continue driving backwards
  leftDrive.spinFor(reverse, 600, deg, 10, velocityUnits::pct, false);
  rightDrive.spinFor(reverse, 600, deg, 10, velocityUnits::pct);
  */
  // Stop the drivetrain not too hard
  leftDrive.stop(coast);
  rightDrive.stop(coast);

  // Clamp onto the goal
  clampPneumatic.set(true);

  // Wait a little bit
  wait(400, msec);
  /*
  // Drive backwards to safety
  leftDrive.spinFor(fwd, 500, deg, 10, velocityUnits::pct, false);
  rightDrive.spinFor(fwd, 500, deg, 10, velocityUnits::pct);
  */
  // Score?!!!!
  intakeUpper.spinFor(3,sec, 70, velocityUnits::pct);
}

// Code run during the driver control period
void usercontrol(void) {

  wait(5,sec);

  // Start the drivetrain (this doesn't necessarily mean it will move)
  leftDrive.spin(fwd);
  rightDrive.spin(fwd);

  // Main loop for driver control code
  while (true == true /*a statement that is true*/) { 
    renderRobot();
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