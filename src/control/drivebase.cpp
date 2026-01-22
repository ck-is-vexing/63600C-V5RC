#include "control/drivebase.h"
#include <cmath>
#include "definition.h"

using namespace vex;

namespace {
  constexpr double WHEEL_DIAMETER = 2.75;
  constexpr double GEAR_RATIO = 36.0 / 48.0;

  constexpr double INCH_CONVERSION = (M_PI * WHEEL_DIAMETER) * GEAR_RATIO / 360.0;
}

Drivebase::Drivebase(vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::brain& robotBrain, vex::inertial& inertialSensor, vex::gps& GPSSensor) 
: ld(leftMotors), rd(rightMotors), br(robotBrain), inert(inertialSensor), gps(GPSSensor),
  headingPID(0.5, 0.0001, 25, 10, true),
  fancyDrivePID(1, 0, 0, 40), // 40ms because of gps refresh rate
  drivePID(0.8, 0, 0, 10) {}

void Drivebase::drive(directionType direction, double inches, int velocityPercent) {

  double motorDegrees = inches / (INCH_CONVERSION);

  ld.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct, false);
  rd.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct);

  ld.stop(brake);
  rd.stop(brake);
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

      if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
          (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) ||
          (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){

        break;
      }
      
      wait(10, msec); 
    }

    ld.stop(hold);
    rd.stop(hold);
    headingPID.reset();

    // In case drivetrain is still in motion and moves away from setpoint
    wait(100, msec);

    currentAngle = inert.heading();

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

void Drivebase::driveTo(double desiredX, double desiredY, double precision, double secondsAllowed, int recursions) {

  // Calculate the angle
  // pv = sqrt((y2 - y1)^2 + (x2 - x1)^2)
  // theta = atan((y2 - y1) / (x2 - x1))
  // Turn to face the desired point
  turnTo(atan(3));

  headingPID.reset();
  fancyDrivePID.reset();
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