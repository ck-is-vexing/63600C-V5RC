#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "func/PID.h"

/**
 * @class drivebase
 * @brief Rewrite later
 * 
 * Rewrite later
 */
class drivebase {
  private:
    PID headingPID;
    PID fancyDrivePID;
    PID drivePID;

    vex::motor_group& ld;
    vex::motor_group& rd;
    vex::brain& br;
    vex::inertial& inert;
    vex::gps& gps;

    const double xOffsetGPS;
    const double yOffsetGPS;
  public:

    /**
     * @brief Rewrite later
     * 
     * Rewrite later
     * @param leftDrivetrain yes
     * @param rightDrivetrain yes
     */
    drivebase(vex::motor_group& leftDrivetrain, vex::motor_group& rightDrivetrain, vex::brain& robotBrain, vex::inertial& inertialSensor, vex::gps& GPSSensor);

    void drive(vex::directionType direction, double inches, int velocityPercent = 30);

    void turnTo(double desiredAngle, double precision = 0.5, double secondsAllowed = 2, int recursions = 5, double minimumSpeed = 1);

    void driveTo(double desiredInches, vex::directionType direction, double desiredAngle, double precision = 0.5, double secondsAllowed = 10, int recursions = 5);

    void posDriveTo(double desiredX, double desiredY, double precision = 0.5, double secondsAllowed = 10, int recursions = 5);

    double get_x();

    double get_y();

    void renderRobot();
};

#endif