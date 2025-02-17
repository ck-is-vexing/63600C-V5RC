#include "vex.h"
#include "control/drivebase.h"
#include <cmath>
#include <iostream>
using namespace vex;

drivebase::drivebase(vex::motor_group& leftDrivetrain, vex::motor_group& rightDrivetrain, vex::brain& robotBrain, vex::inertial& inertialSensor, vex::gps& GPSSensor) 
: ld(leftDrivetrain), rd(rightDrivetrain), br(robotBrain), inert(inertialSensor), gps(GPSSensor),
  headingPID(0.55, 0, 30, 10, true), // 0.6, 0, 28 initially
  fancyDrivePID(1, 0, 0, 40), // 40ms because of gps update rate
  drivePID(0.01, 0, 40, 10),
  xOffsetGPS(2), // In inches
  yOffsetGPS(-5.5) {} // In inches

// Not-so-pid drive. Takes a VEX direction and the distance in inches. Optionally add a specific velocity
void drivebase::drive(directionType direction, double inches, int velocityPercent){

  //Conversion from inches to degrees of rotation for the motor
  double motorDegrees = inches / (0.0170169602069); // inches divided by (Circumference of the wheels) * (The gear ratio of the robot) / (360, to put the number into degrees)

  ld.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct, false);
  rd.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct);
}

// PID enabled turn function
// desiredAngle is the setpoint
void drivebase::turnTo(double desiredAngle, double precision, double secondsAllowed, int recursions, double minimumSpeed) {

  // Reset the PID
  headingPID.reset();

  // vars
  double currentAngle;

  // Loop of recursions
  for (int i = 0; i < recursions; i++){

    // Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){

      // Update angle
      currentAngle = inert.heading();
      double change = headingPID.update(desiredAngle, currentAngle);
      
      // Force the amount of change to be greater than the minimum speed, so that the robot actually makes it to the setpoint...
      // This is more consistent than purely using the integral term, as it never affects the system except for at extremely low speed
      // With just the integral, the tuning would only ever either make the robot overshoot or not do enough to actually matter
      if (std::abs(change) < minimumSpeed) {
        change = std::copysign(minimumSpeed, change);
      }

      // Spin drivetrain
      ld.spin(fwd,change,pct);
      rd.spin(reverse,change,pct);

      // Check for completion of the tick loop
      if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
        (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) || // Check for completion at a near 0 or 360 degree angle
        (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){ // Same as above

        // Break the tick loop
        break;
      }

      // Prevent wasted resources by waiting a short period of time before iterating
      wait(10,msec); 
    }

    ld.stop(brake);
    rd.stop(brake);

    // Wait a little bit in case the system is still in motion
    wait(300,msec);

    // Update currentAngle
    currentAngle = inert.heading();

    // Check if desired angle was achieved
    if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
        (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) || // Check for completion at a near 0 or 360 degree angle
        (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){ // Same as above
    
      // Stop drivetrain
      ld.stop(coast);
      rd.stop(coast);

      // Break the recursion loop and the whole turnTo, because it has reached the setpoint
      break;
    }  
    // If the correct angle was not achieved, the code will recurse
  }
}

// PID enabled drive function without GPS
// Takes an angle and a distance
// Has issues due to motor angle data not being precise enough--should be switched to tracking wheels
void drivebase::driveTo(double desiredInches, vex::directionType direction, double desiredAngle, double precision, double secondsAllowed, int recursions) {

  // Reset the PIDs
  headingPID.reset();
  drivePID.reset();

  // Reset the drivetrain sensor positions
  ld.setPosition(0, deg);
  rd.setPosition(0, deg);

  // variables
  double currentDistance = 0;
  double currentAngle;
  double oldDeg = (ld.position(degrees) + rd.position(degrees)) / 2; // Averages the position of the two sides, in order to cancel out drift
  double currentDeg;

  // Loop of recursions
  for (int i = 0; i < recursions; i++){

    // Loop of ticks, converting seconds allowed into sets of 10 ms
    for (int t = 0; t < (secondsAllowed * 100); t++){

      // Update angle
      currentAngle = inert.heading();
      double change = headingPID.update(desiredAngle, currentAngle);

      // Update distance
      // Distance is achieved without using the GPS by taking the integral of the derivative of motor position and converting to inches
      // Future optimizations include removing calculating the derivative by just taking the motor.velocity() directly
      currentDeg = (ld.position(degrees) + rd.position(degrees)) / 2;
      currentDistance += (currentDeg - oldDeg) * (0.0170169602069);
      oldDeg = currentDeg;
      std::cout << currentDistance << std::endl;

      change += drivePID.update(desiredInches, currentDistance);

      // Spin drivetrain
      ld.spin(direction,change,pct);
      rd.spin(direction,change,pct);

      // Check for completion of the tick loop
      if (currentDistance < (desiredInches + precision) && currentDistance > (desiredInches - precision)){ //Same as above

        // Break the tick loop
        break;
      }

      // Prevent wasted resources by waiting a short period of time before iterating
      wait(10,msec); 
    }

    ld.stop(brake);
    rd.stop(brake);

    // Wait a little bit in case the system is still in motion
    wait(300,msec);

    // Update currentAngle
    currentAngle = inert.heading();

    // Check if desired angle was achieved
    if (currentDistance < (desiredInches + precision) && currentDistance > (desiredInches - precision)){ //Same as above
    
      // Stop drivetrain using coast
      ld.stop(coast);
      rd.stop(coast);

      // Break the recursion loop and the whole turnTo, because it has reached the setpoint
      break;
    }
    // If the correct angle was not achieved, the code will recurse
  }
}

// PID enabled drive function using the location of the robot
// desiredX and desiredY are coordinates on the VEX Field, with (0,0) being the red negative corner, and the unit being inches.
// Does not work right now silly! 
void drivebase::posDriveTo(double desiredX, double desiredY, double precision, double secondsAllowed, int recursions) {

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
      currentAngle = inert.heading();
      double change = headingPID.update(desiredX, currentAngle);

      // Spin drivetrain
      ld.spin(fwd,change,pct);
      rd.spin(reverse,change,pct);

      // Check for completion of the tick loop
      if (currentAngle < (desiredX + precision) && currentAngle > (desiredX - precision)){

        // Break the tick loop
        break;
      }

      // Prevent wasted resources by waiting a short period of time before iterating
      wait(10,msec); 
    }

    ld.stop(brake);
    rd.stop(brake);

    // Wait a little bit in case the system is still in motion
    wait(200,msec);

    // Update currentAngle
    currentAngle = inert.heading();

    // Check if desired angle was achieved
    if (currentAngle < (desiredX + precision) && currentAngle > (desiredX - precision)){

      // Stop drivetrain
      ld.stop(coast);
      rd.stop(coast);

      // Break the recursion loop and the whole turnTo, because it has reached the setpoint
      break;
    }  

    // If the correct angle was not achieved, the code will recurse
  }
}

// Returns the x position of the robot center in inches
double drivebase::get_x() {
  
  double theta = 180 - atan(xOffsetGPS / yOffsetGPS) - (inert.heading() * M_PI / 180);
  double deltaX = cos(theta) * sqrt(xOffsetGPS * xOffsetGPS + yOffsetGPS * yOffsetGPS);

  return (gps.xPosition(distanceUnits::in) + deltaX);
}

// Returns the y position of the robot center in inches
double drivebase::get_y() {

  double theta = 180 - atan(xOffsetGPS / yOffsetGPS) - (inert.heading() * M_PI / 180);
  double deltaY = sin(theta) * sqrt(xOffsetGPS * xOffsetGPS + yOffsetGPS * yOffsetGPS);

  return (gps.yPosition(distanceUnits::in) + deltaY);
}

// Renders an approximation of the robot position on the Brain screen
void drivebase::renderRobot() {

  // VEX Brain is 480x240p
  // 20p = 1 foot
  // GPS sensor (0,0) is at the center of the field
  br.Screen.clearScreen();
  br.Screen.setPenColor(white);
  br.Screen.setPenWidth(2);

  br.Screen.setFillColor(yellow);
  br.Screen.drawCircle(120,120,8);

  // Draw field
  for(int x = 0; x <= 6; x++){
    br.Screen.drawLine(x * 40, 0, x * 40, 240);
  }

  for(int y = 0; y <= 6; y++){
    br.Screen.drawLine(0, y * 40, 240, y * 40);
  }

  // Draw robot vector
  double angle = inert.heading() * M_PI / 180; // Get angle in radians
  double x = get_x() * 5/3; // Multiplied by 5/3 to convert inches to pixels
  double x_gps = gps.xPosition(distanceUnits::in) * 5/3;
  double x2 = x + cos(angle) * 40; // 40 is the desired length of the render of the vector
  double y = get_y() * 5/3;
  double y_gps = gps.yPosition(distanceUnits::in) * 5/3;
  double y2 = y + sin(angle) * 40;

  br.Screen.setPenColor(blue);
  br.Screen.setFillColor(blue);
  br.Screen.drawCircle(x_gps, y_gps, 8);

  br.Screen.setPenColor(red);
  br.Screen.setFillColor(red);
  br.Screen.setPenWidth(6);
  br.Screen.drawCircle(x + 120, y + 120, 8);
  br.Screen.drawLine(x + 120, y + 120, x2 + 120, y2 + 120);

  br.Screen.setCursor(4, 30);
  br.Screen.setPenColor(white);
  br.Screen.setFillColor(black);
  br.Screen.setPenWidth(1);
  br.Screen.print("Angle: ");
  br.Screen.print(angle);
  br.Screen.setCursor(5, 30);
  br.Screen.print("X: ");
  br.Screen.print(x_gps);
  br.Screen.setCursor(6, 30);
  br.Screen.print("Y: ");
  br.Screen.print(y_gps);
}