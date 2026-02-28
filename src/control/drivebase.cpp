#include "control/drivebase.h"

#include "definition.h"
#include <cmath>

using namespace vex;

namespace {
  constexpr double WHEEL_DIAMETER  = 2.75;
  constexpr double GEAR_RATIO      = 36.0 / 48.0;
  constexpr double INCH_CONVERSION = (M_PI * WHEEL_DIAMETER) * GEAR_RATIO / 360.0;

  constexpr double RAD_TO_DEG      = (180  / M_PI);
  constexpr double DEG_TO_RAD      = (M_PI / 180);

  constexpr double ANGLE_COORDINATE_TRANSFORMATION = (3*M_PI/2);
}

Drivebase::Drivebase(vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::brain& robotBrain, vex::inertial& inertialSensor, vex::gps& GPSSensor) 
: ld(leftMotors), rd(rightMotors), br(robotBrain), inert(inertialSensor), gps(GPSSensor),
  headingPID(0.55, 0.0001, 20, 10, true), // 0.57 0 16
  drivePID(2.8, 0.0002, 10, 10) {} // 0.0001 for skills?

void Drivebase::drive(vex::directionType direction, double inches, int velocityPercent, vex::brakeType stoppingType) {

  double motorDegrees = inches / (INCH_CONVERSION);

  ld.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct, false);
  rd.spinFor(direction, motorDegrees, deg, velocityPercent, velocityUnits::pct);

  ld.stop(stoppingType);
  rd.stop(stoppingType);
}

void Drivebase::drive(vex::directionType direction, int velocity, vex::velocityUnits velocityUnit) {
  ld.spin(fwd, velocity, velocityUnit);
  rd.spin(fwd, velocity, velocityUnit);
}


void Drivebase::stop() {
  ld.stop(coast);
  rd.stop(coast);
}

void Drivebase::stop(vex::brakeType brakingType) {
  ld.stop(brakingType);
  rd.stop(brakingType);
}


void Drivebase::turnTo(double desiredAngle, double precision, double secondsAllowed, int recursions) {
  constexpr double MINIMUM_SPEED = 2.5;
  
  headingPID.reset();
  double currentAngle;

  for (int i = 0; i < recursions; i++) {
    for (int t = 0; t < (secondsAllowed * 100); t++) { // Loops in sets of 10 ms

      currentAngle = inert.heading();
      double speed = headingPID.update(desiredAngle, currentAngle);
      
      // Represents the realistic abilities of VEX motors
      if (std::abs(speed) <= MINIMUM_SPEED) {
        speed = std::copysign(MINIMUM_SPEED, speed);
      }

      ld.spin(fwd, speed, pct);
      rd.spin(reverse, speed, pct);

      if ((currentAngle < (desiredAngle + precision)       && currentAngle > (desiredAngle - precision))       ||
          (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) ||
          (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ) {

        break;
      }
      
      wait(10, msec); 
    }

    ld.stop(brake);
    rd.stop(brake);
    headingPID.reset();

    // In case drivetrain is still in motion and moves away from setpoint
    wait(50, msec);

    currentAngle = inert.heading();

    if ((currentAngle < (desiredAngle + precision)       && currentAngle > (desiredAngle - precision))       ||
        (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) ||
        (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ) {
    
      ld.stop(brake);
      rd.stop(brake);
      break;
    }  
    // If the correct angle was not achieved, the code will recurse
  }
}

void Drivebase::pointTo(pose::Pos desiredPoint, double precision, double secondsAllowed, int recursions) {

  pose::Pose curPose  =  pose::odom::getPose();
  double desiredAngle = (ANGLE_COORDINATE_TRANSFORMATION - atan2( (desiredPoint.y - curPose.y), (desiredPoint.x - curPose.x) )) * RAD_TO_DEG;

  desiredAngle        = std::remainder(desiredAngle, 360);

  Drivebase::turnTo(desiredAngle, precision, secondsAllowed, recursions);
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

void Drivebase::driveTo(pose::Pos desiredPos, pose::Pos precision, double secondsAllowed, int recursions, double minimumSpeed) {
  constexpr double ANGLE_SPEED_MODIFIER            = 1.5;

  constexpr double REVERSAL_THRESHOLD_ANGLE        = (3*M_PI/4);


  headingPID.reset();
  drivePID.reset();
  pose::Pose curPose;

  for (int i = 0; i < recursions; i++) {
    for (int t = 0; t < (secondsAllowed * 100); t++) {

      curPose             = pose::odom::getPose();
      double deltaX       = (desiredPos.x - curPose.x);
      double deltaY       = (desiredPos.y - curPose.y);

      double pv           = sqrt( (deltaY * deltaY) + (deltaX * deltaX) );
      double angleToPoint = (ANGLE_COORDINATE_TRANSFORMATION - atan2( deltaY, deltaX ));


      angleToPoint = std::fmod(angleToPoint, 2*M_PI);

      if (angleToPoint < 0) {
        angleToPoint     += 2*M_PI;
      }


      int reversalMod     = 1;
      double angleDiff    = std::remainder( (angleToPoint - curPose.theta), M_PI*2 );
      
      if (std::abs(angleDiff)  >= REVERSAL_THRESHOLD_ANGLE) {
        reversalMod       = -1;
        angleToPoint     -= std::copysign(M_PI, (angleToPoint - curPose.theta));
      }

      //printl(angleToPoint << "     " << curPose.theta << "     " << angleDiff << "    " << reversalMod);


      double angleSpeed   = headingPID.update((angleToPoint * RAD_TO_DEG), (curPose.theta * RAD_TO_DEG));
      double driveSpeed   = drivePID.update(0, -pv);
      
      // Represents the realistic abilities of VEX motors
      /*if (std::abs(driveSpeed) < minimumSpeed) {
        driveSpeed = std::copysign(minimumSpeed, driveSpeed);
      }*/

      ld.spin(fwd, ((driveSpeed * reversalMod) + (angleSpeed * ANGLE_SPEED_MODIFIER)), pct);
      rd.spin(fwd, ((driveSpeed * reversalMod) - (angleSpeed * ANGLE_SPEED_MODIFIER)), pct);


      if ((pv * pv) < (precision.x * precision.y)) { break; }
      
      wait(10, msec); 
    }


    ld.stop(brake);
    rd.stop(brake);
    headingPID.reset();
    drivePID.reset();

    // In case drivetrain is still in motion and moves away from setpoint
    wait(50, msec);

    double deltaX    = (desiredPos.x - curPose.x);
    double deltaY    = (desiredPos.y - curPose.y);
    
    curPose          =  pose::odom::getPose();
    double pvSquared = (deltaY * deltaY) + (deltaX * deltaX);

    if (pvSquared < (precision.x * precision.y)) {
      ld.stop(brake);
      rd.stop(brake);
      break;
    }
  }
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