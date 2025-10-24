#include "control/drivebase.h"
#include <cmath>
#include "definition.h"

using namespace vex;

namespace {
  constexpr double WHEEL_DIAMETER = 2.75;
  constexpr double GEAR_RATIO = 36.0 / 48.0;

  constexpr double INCH_CONVERSION = (M_PI * WHEEL_DIAMETER) * GEAR_RATIO / 360.0;

  constexpr double X_OFFSET_GPS_INCHES = 2;
  constexpr double Y_OFFSET_GPS_INCHES = -5.5;
}

Drivebase::Drivebase(vex::motor_group& leftDrivetrain, vex::motor_group& rightDrivetrain, vex::brain& robotBrain, vex::inertial& inertialSensor, vex::gps& GPSSensor) 
: ld(leftDrivetrain), rd(rightDrivetrain), br(robotBrain), inert(inertialSensor), gps(GPSSensor),
  headingPID(0.5, 0.0001, 25, 10, true),
  fancyDrivePID(1, 0, 0, 40), // 40ms because of gps refresh rate
  drivePID(0.8, 0, 0, 10) {}

void Drivebase::drive(directionType direction, double inches, int velocityPercent){

  double motorDegrees = inches / (INCH_CONVERSION); // 0.018 as a backup

  ld.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct, false);
  rd.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct);
}

void Drivebase::turnTo(double desiredAngle, double precision, double secondsAllowed, int recursions, double minimumSpeed) {
  
  headingPID.reset();
  double currentAngle;

  for (int i = 0; i < recursions; i++){
    for (int t = 0; t < (secondsAllowed * 100); t++){ // Loops in sets of 10 ms

      currentAngle = inert.heading();
      double change = headingPID.update(desiredAngle, currentAngle);
      
      // Represents the realistic abilities of VEX motors
      if (std::abs(change) < minimumSpeed) {
        change = std::copysign(minimumSpeed, change);
      }

      ld.spin(fwd, change, pct);
      rd.spin(reverse, change, pct);

      // Check for completion of the tick loop
      if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
          (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) ||
          (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){

        break;
      }
      
      wait(10, msec); 
    }

    ld.stop(brake);
    rd.stop(brake);
    headingPID.reset();

    // In case drivetrain is still in motion and moves away from setpoint
    wait(100, msec);

    currentAngle = inert.heading();

    // Check if desired angle was achieved
    if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
        (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) ||
        (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){
    
      ld.stop(coast);
      rd.stop(coast);
      break;
    }  
    // If the correct angle was not achieved, the code will recurse
  }
}

void Drivebase::driveTo(vex::directionType direction, double desiredInches, double desiredAngle, double precision, double secondsAllowed, int recursions, double minSpeed) {

  headingPID.reset();
  drivePID.reset();
  ld.setPosition(0, deg);
  rd.setPosition(0, deg);

  double currentDistance = 0;
  double currentAngle;
  double oldDeg = (ld.position(degrees) + rd.position(degrees)) / 2; // Averages the position of the two sides, in order to cancel out drift
  double currentDeg;

  for (int i = 0; i < recursions; i++){
    for (int t = 0; t < (secondsAllowed * 100); t++){ // Loops in sets of 10 ms

      currentAngle = inert.heading();
      double angleChange = headingPID.update(desiredAngle, currentAngle);

      // Takes the integral of the derivative of motor position and converts to inches. Required because motor position is measured 0-360 degrees
      currentDeg = (ld.position(degrees) + rd.position(degrees)) / 2;
      currentDistance += (currentDeg - oldDeg) * INCH_CONVERSION;
      oldDeg = currentDeg;
      
      double driveChange = drivePID.update(desiredInches, currentDistance);
      
      // Represents the realistic abilities of VEX motors
      if (std::abs(driveChange) < minSpeed) {
        driveChange = std::copysign(minSpeed, driveChange);
      }

      ld.spin(fwd, angleChange + driveChange, pct);
      rd.spin(fwd, -angleChange + driveChange, pct);

      // Check for completion of the tick loop
      if (currentDistance < (desiredInches + precision) && currentDistance > (desiredInches - precision)){
        break;
      }

      wait(10,msec); 
    }

    ld.stop(brake);
    rd.stop(brake);

    // In case drivetrain is still in motion and moves away from setpoint
    wait(200,msec);

    currentAngle = inert.heading();

    // Check if desired location was achieved
    if (currentDistance < (desiredInches + precision) && currentDistance > (desiredInches - precision)){
    
      ld.stop(coast);
      rd.stop(coast);
      break;
    }
    // If the correct location was not achieved, the code will recurse
  }
}

void Drivebase::posDriveTo(double desiredX, double desiredY, double precision, double secondsAllowed, int recursions) {

  // Calculate the angle
  // pv = sqrt((y2 - y1)^2 + (x2 - x1)^2)
  // theta = atan((y2 - y1) / (x2 - x1))
  // Turn to face the desired point
  turnTo(atan(3));

  headingPID.reset();
  fancyDrivePID.reset();
}

double Drivebase::get_x() const {
  
  double theta = 180 - atan(X_OFFSET_GPS_INCHES / Y_OFFSET_GPS_INCHES) - (inert.heading() * M_PI / 180);
  double deltaX = cos(theta) * sqrt(X_OFFSET_GPS_INCHES * X_OFFSET_GPS_INCHES + Y_OFFSET_GPS_INCHES * Y_OFFSET_GPS_INCHES);

  return (gps.xPosition(distanceUnits::in) + deltaX);
}

double Drivebase::get_y() const {

  double theta = 180 - atan(X_OFFSET_GPS_INCHES / Y_OFFSET_GPS_INCHES) - (inert.heading() * M_PI / 180);
  double deltaY = sin(theta) * sqrt(X_OFFSET_GPS_INCHES * X_OFFSET_GPS_INCHES + Y_OFFSET_GPS_INCHES * Y_OFFSET_GPS_INCHES);

  return (gps.yPosition(distanceUnits::in) + deltaY);
}

void Drivebase::renderRobot() {
  //TODO: Could the trig code be mixing up rad and deg?

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
  constexpr int VECTOR_LENGTH_PIXELS = 40;

  double angle = inert.heading() * M_PI / 180;
  double x = get_x() * 5/3; // Convert inches to pixels
  double x_gps = gps.xPosition(distanceUnits::in) * 5/3;
  double x2 = x + cos(angle) * VECTOR_LENGTH_PIXELS;
  double y = get_y() * 5/3;
  double y_gps = gps.yPosition(distanceUnits::in) * 5/3;
  double y2 = y + sin(angle) * VECTOR_LENGTH_PIXELS;

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

void Drivebase::demoTo(double desiredAngle) {

  headingPID.reset();
  double currentAngle;

  while (true) {

    currentAngle = inert.heading();
    double change = headingPID.update(desiredAngle, currentAngle);

    ld.spin(fwd, change, pct);
    rd.spin(reverse, change, pct);
    
    wait(10, msec); 
  }
}