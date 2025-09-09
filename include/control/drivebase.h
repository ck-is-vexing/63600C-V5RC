#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "func/PID.h"

// TODO: add comments
/**
 * @class drivebase
 * @brief Rewrite later
 * 
 * Rewrite later
 */
class drivebase {
  private:
    vex::motor_group& ld;
    vex::motor_group& rd;
    vex::brain& br;
    vex::inertial& inert;
    vex::gps& gps;

    PID headingPID;
    PID fancyDrivePID;
    PID drivePID;

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

    void drive(vex::directionType direction, double inches, int velocityPercent = 60);

    void turnTo(double desiredAngle, double precision = 0.5, double secondsAllowed = 2, int recursions = 5, double minimumSpeed = 1.8);

    void driveTo(vex::directionType direction, double desiredInches, double desiredAngle, double precision = 0.1, double secondsAllowed = 10, int recursions = 5, double minSpeed = 1.0);

    void posDriveTo(double desiredX, double desiredY, double precision = 0.5, double secondsAllowed = 10, int recursions = 5);

    double get_x();

    double get_y();

    void renderRobot();

    void demoTo(double desiredAngle, double precision = 0.5, double secondsAllowed = 2000, int recursions = 5, double minimumSpeed = 0);
};

#endif