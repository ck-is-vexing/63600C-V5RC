// *******************************************************************************
// **********************************Setup****************************************
// *******************************************************************************

#include "vex.h" // Include the vex header
using namespace vex; // Set the namespace to vex

// A global instance of competition
competition Competition;


// *******************************************************************************
// *******************************Global Vars*************************************
// *******************************************************************************

bool skillsMode = true; // A toggle that says whether to run this as the game program or skills program. This is to make it easy to switch between them when uploading code.
bool gpsAllowed = true; // A toggle for whether the field has GPS strips or not
bool farSide = false; // A toggle to tell where the robot is starting on the field

double xOffsetGPS = -6; // In inches
double yOffsetGPS = 5; // In inches


// *******************************************************************************
// ********************************Functions**************************************
// *******************************************************************************

// ---------------------------Responsive Functions---------------------------------

// Function that checks controller inputs and responds
void checkInputs(){
  leftDrive.setVelocity(Controller1.Axis3.value(),pct);
  rightDrive.setVelocity(Controller1.Axis2.value(),pct);

}


//------------------------Autonomous Support Functions----------------------------

//Function to drive forwards or backwards. This is required because motor groups have to be used instead of a drivetrain in the code due to the 6 motor drivetrain.
void drive(char dir, float inches, int velocityPercent = 20){

  //Conversion from inches to degrees of rotation for the motor.
  double motorDegrees = inches / (1/*insert correct number here*/); //(Circumphrence of the wheels) / (The gear ratio of the robot) / (360, to put the number into degrees). The number was shortened to save processing power.

  if (dir == 'f'){ //Run code in the curly brackets if the user specified 'f', or forwards.
    // ~~insert drive code here?
  } else if (dir == 'b' || dir == 'r'){ //This could be replaced with just an else statement as these are the only 3 options. It hasn't because then there couldn't be an error response.
    // ~~insert drive code here?
  } else {
    // do nothing and cry because whoever tried to call this function messed up
  }
}

//Function to turn the robot clockwise or counterclockwise. This is required because motor groups have to be used instead of a drivetrain in the code due to the 6 motor drivetrain.
void turn(char dir, float degrees, int velocityPercent = 20){ 

  //"https://www.desmos.com/calculator/ovrk4bnpat" for an interactive and easier to follow version of this math.
  double robotTurnRadius = 4.816; //The distance between the centers of two opposite robot wheels divided by 2. Essentially the radius of the circle that the robot turns around.
  double robotArc = robotTurnRadius * (degrees * 3.1415 / 180); //The length of the arc along the turning radius that the robot is intended to follow.
  double motorDegrees = robotArc / (3.25 * 3.1415) * 600; //This equates the final amount of degrees that the motor groups must rotate. It is calculated with the arc length variable divided by the circumphrence of the robot wheels. Then it is multiplied by 600, a number that both converts to degrees and accounts for the gear ratio.

  if (dir == 'l'){ //If the robot is turning left, run the below code
    // .
  } else if (dir == 'r'){ //If the robot is turning right, run the below code.
    // ..
  } else { // this *should* never be called
    // ...
  }
}

//Inertial sensor calibration function using GPS. This ends up being more precise than setting the robot up the same direction every time.
void inertialGPSCalibrate(double averageSeconds = 1){

  //For the robot to not average the turn at all, set seconds to 0.

  //Setup variables
  double averagedHeading = 0; //The final averaged heading
  int i; //A counter for the for loop. This must be defined outside of it for the final for loop.

  //For loop adds up all the different angles of the GPS. Step one of averaging the robot angle.
  for(i = 0; i < (averageSeconds * 25) + 1; i++){

    //Add the GPS heading to the total
    averagedHeading += GPS.heading();

    //Wait a short period because the GPS only updates a certain amount of times a second
    wait(40,msec);
  }

  //Divide averagedHeading by the amount of GPS headings added to it
  averagedHeading /= i;

  //Log the averaged heading versus the GPS heading
  Controller1.Screen.clearScreen(); //Clear any previous text
  Controller1.Screen.setCursor(1, 1); //Set the cursor to the start of the first row
  Controller1.Screen.print("Average: "); //Print a label for the averaged heading
  Controller1.Screen.print(averagedHeading); //Print the averaged heading
  Controller1.Screen.setCursor(2, 1); //Set the cursor down a row
  Controller1.Screen.print("GPS: "); //Print a label for the GPS heading
  Controller1.Screen.print(GPS.heading()); //Print the GPS heading

  //Set the inertial sensor's heading to the GPS heading.
  Inertial.setHeading(averagedHeading, deg);

  //Loop until the inertial sensor finishes setting the heading
  while(Inertial.isCalibrating()){

    //Wait to prevent wasted resources by iterating too fast
    wait(100,msec);
  }

  //Let the user know that calibration is complete
  Controller1.Screen.setCursor(3, 1); //Set the cursor to the next available row
  Controller1.Screen.print("Inertial Calibrated!"); //Print a nice message for the driver

}

//PI Conrolled turn function using inertial sensor
void turnTo(double desiredAngle, double precision = 0.3, double secondsAllowed = 2, int recursions = 5){
  
  /*  
    Explanation of input variables:

    desiredAngle: The angle that the programmer desires the robot to turn to.
    precision: How close the robot must be to the desired angle for it to quit the turn.
    secondsAllowed: A cap for how long the turn code is allowed to repeat for each recurson. This is useful to make the robot never get stuck in an infinite loop of incorrectness
    recursions: The amount of seperate times the robot should be allowed to pass desiredAngle and continue to turn towards it.
  */

  //Variables
  double robotAngle = 0; //The current angle of the robot, as a variable
  double error = 0; //How far the robot is from the intended angle, calculated with the difference of desiredAngle and robotAngle.
  double motorVelocity = 0; //The velocity the drivetrain motors are set to
  double integral = 0; //The integral parameter

  //Force the desired angle to be within 360 degrees by using mod
  desiredAngle = fmod(desiredAngle,360); 

  //Loop of recursions
  for (int i = 0; i < recursions; i++){

    //Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){
      
      //Update robotAngle
      robotAngle = Inertial.heading();

      //How far the robot is from the intended angle, calculated with the difference of desiredAngle and robotAngle.
      error = desiredAngle - robotAngle; 

      //Find the shortest possible route to reach target angle
      if (error > 180){
        error -= 360;
      } else if (error < -180){
        error += 360;
      }
      
      //Update Integral
      integral += (error * t / 1000); //Integral = Integral + (current error) * (tick number, one every 10 milliseconds) / (1000, to convert to seconds)

      //Calculating the final velocity for motors
      motorVelocity = (0.45 * error) + (0.05 * integral); //The first set of parenthesis contains Proportional control; the second Integral control. Each is multiplied by a tuning variable.

      //Set the motor groups to spin at the speed of motorVelocity
      // ...

      //Check for completion of the loop of ticks
      if ((robotAngle < (desiredAngle + precision) && robotAngle > (desiredAngle - precision)) ||
        (robotAngle < (desiredAngle + precision - 360) && robotAngle > (desiredAngle - precision - 360)) || //Check for completion at a near 0 or 360 degree angle
        (robotAngle < (desiredAngle + precision + 360) && robotAngle > (desiredAngle - precision + 360)) ){ //Same as above
        
        //Break the tick loop
        break;
      }

      //Prevent wasted resources by waiting a short period of time before iterating
      wait(10,msec); 
    }

    //Stop motors
    // ...

    //Reset integral
    integral = 0;

    //Wait a little bit in case the robot is still in motion
    wait(200,msec);

    //Update robotAngle
    robotAngle = Inertial.heading();

    //Check if desired angle was achieved
    if ((robotAngle < (desiredAngle + precision) && robotAngle > (desiredAngle - precision)) ||
      (robotAngle < (desiredAngle + precision - 360) && robotAngle > (desiredAngle - precision - 360)) || //Check for completion at a near 0 or 360 degree angle
      (robotAngle < (desiredAngle + precision + 360) && robotAngle > (desiredAngle - precision + 360)) ){ //Same as above

      //Update controller screen to tell user the robot has finished the turn.
      Controller1.Screen.clearScreen(); //Clear the scren
      Controller1.Screen.setCursor(1, 1); //Set the cursor to the first line of the controller
      Controller1.Screen.print("Turn Complete!"); //Print a nice message
      Controller1.Screen.setCursor(2, 1); //Set the cursor to the second line
      Controller1.Screen.print("Angle: "); //Print a label for robotAngle
      Controller1.Screen.print(robotAngle); //Print robotAngle to show if the code encountered a bug

      //Set the motors to stop on coast
      // ...

      //Break the recursion loop and the whole turnTo, because it has reached the desired angle
      break;
    }  

    //If the correct angle was not achieved, the code will recurse
  }
}

//PI Controlled aim function using inertial sensor
void aimTo(double desiredAngle, double aimTime, double defaultMovementPercent = 0, double integralResetPrecision = 0.1, int maxIntegralUpdates = 500){

  /*  
    Explanation of input variables:

    desiredAngle: The heading that the robot should aim towards
    aimTime: The amount of time in seconds that the robot should aim towards desiredAngle
    defaultMovementPercent: A motor percentage that the robot should automatically move forwards or back. This is needed because the catapult this season slowly brings the robot forwards, away from the match load bar.
    integralResetPrecision: How close the robot needs to be to the desired angle to reset the integral. Without resetting the integral, the controller would mess up since it needs to aim for a certain amount of time.
    maxIntegralUpdates: The maximum amount of time the integral can go without being reset. Divide this number by 100 to get it in seconds.
  */

  //Aim setup
  double robotAngle = 0; //The current angle of the robot, as a variable
  double error = 0; //How far the robot is from the intended angle
  double motorVelocity = 0; //The velocity the drivetrain motors are set to
  double integral = 0; //The integral parameter
  double timeCount = 0; //A counter for time

  //Force the desired angle to be within 360 degrees by using mod
  desiredAngle = fmod(desiredAngle,360); 

  //Timed loop
  while(timeCount < aimTime){ //While the variable counting time is less than the amount of time the robot should aim for, repeat.

    //Main PID loop
    for (int t = 0; t < maxIntegralUpdates; t++){ //Repeat until t becomes greater than maxIntegralUpdates.

      //=========================Aim towards the correct angle=========================
      
      //Update robotAngle
      robotAngle = Inertial.heading();

      //How far the robot is from the intended angle, calculated with the difference of desiredAngle and robotAngle.
      error = desiredAngle - robotAngle; 

      //Find the shortest possible route to reach the target angle
      if (error > 180){
        error -= 360;
      } else if (error < -180){
        error += 360;
      }
      
      //Update Integral
      integral += (error * t / 1000); //Integral = Integral + (current error) * (tick number, one every 10 milliseconds) / (1000, to convert to seconds)

      //Calculating the final velocity for motors
      motorVelocity = (0.8 * error) + (0.03 * integral); //The first set of parenthesis contains the proportional variable; the second contains the integral. Each is multiplied by a tuning number.

      //Set the motor groups to spin at the speed of motorVelocity, combined with the default movement. Default movement is needed to stay in contact with the match load bar.
      // ...
      //Add to timeCount in seconds
      timeCount += 0.01;

      //Prevent wasted resources by waiting a short period of time
      wait(10,msec); 

      //Check for integral reset
      if ((robotAngle < (desiredAngle + integralResetPrecision) && robotAngle > (desiredAngle - integralResetPrecision)) ||
      (robotAngle < (desiredAngle + integralResetPrecision - 360) && robotAngle > (desiredAngle - integralResetPrecision - 360)) || //Check for completion at a near 0 or 360 degree angle
      (robotAngle < (desiredAngle + integralResetPrecision + 360) && robotAngle > (desiredAngle - integralResetPrecision + 360)) || //Same as above
      (timeCount < aimTime)){ //Check for completion based on time

        //Break the loop to allow for integral reset or to finish the aimTo.
        break;
      }
    }

    //Stop motors
    // ...

    //Reset integral
    integral = 0;
  }

  //Wait a little bit while the motors brake
  wait(200,msec);

}


//------------------------Autonomous Functions----------------------------

// Autonomous function ran at the start of a competition match when the robot is on the far field side.
void farGameAuton(){
  // The season hasn't started yet!
}

// Autonomous function ran at the start of a competition match when the robot is on the close field side
void closeGameAuton(){
  // Same as in farGameAuton!
}

// Autonomous skills run.
void autonSkillsAuton(){
  // Yet another instance of... you guessed it... the season hasn't started yet!
}


//---------------------------Game/Match Functions-----------------------------

//Setup code ran before the competition starts
void pre_auton(void) {
  /*
  //Check if using the GPS is allowed
  if(gpsAllowed == true){

    //Wait a short period to allow the GPS to register field strips
    wait(200,msec);

    //Force a GPS update before the code that needs it beings
    Brain.Screen.print(GPS.heading()); 
    
    //Wait a short period to allow the GPS to update
    wait(200,msec);

    //Set the GPS origin
    GPS.setOrigin(xOffsetGPS, yOffsetGPS, inches);

    //Calibrate the inertial sensor with the custom calibration function with a 0.5 second averaging time
    inertialGPSCalibrate(0.5);
  
  } else {

    //If the robot doesn't have access to GPS strips, assume that it was set up with the GPS sensor facing 180 degrees.
    Inertial.setHeading(180, deg);

    //Loop until the inertial sensor finishes calibrating.
    while(Inertial.isCalibrating()){

      //Wait to prevent wasted resources by iterating too fast
      wait(100,msec);
    }

    //Let the driver know that calibration is complete
    Controller1.Screen.setCursor(3, 1); //Set the cursor to the next available row
    Controller1.Screen.print("Inertial Calibrated!"); //Print a nice message for the driver
  }
  */
  // Set drivetrain motors to coast here?
  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

//Function run during the autonomous period
void autonomous(void) {
  
  //If statement switch uses the variable at the top of the program to determine what auton to run.
  if (skillsMode == true){
    autonSkillsAuton(); //Calls the Autonomous Skills auton
  } else if (farSide == true){
    farGameAuton(); //Call the far side auton
  } else {
    closeGameAuton(); //Call the close side auton
  }
}

//Code run during the driver control period
void usercontrol(void) {
  
  leftDrive.spin(fwd);
  rightDrive.spin(fwd);

  //Main loop for driver control code
  while (true == true /*a statement that is true*/) { 

    checkInputs(); //Calls checkInputs, which checks buttons and joysticks on the controller and responds accordingly
    wait(20, msec); // Sleep the program for a short amount of time to prevent wasted resources
  }
}

// Main sets up callbacks and runs the pre auton
int main() {
  // Set up callbacks for autonomous and driver control periods
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function
  pre_auton();

  // Prevent main from exiting with an infinite loop
  while (true) {
    wait(100, msec);
  }
}