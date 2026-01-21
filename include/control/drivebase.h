#pragma once

#include "vex.h"
#include "func/PID.h"

/**
 * @class Drivebase
 * @brief Initialize a robot drivebase and control it
 * 
 */
class Drivebase {
  private:
    vex::motor_group& ld;
    vex::motor_group& rd;
    vex::brain& br;
    vex::inertial& inert;
    vex::gps& gps;

    PID headingPID;
    PID fancyDrivePID;
    PID drivePID;
    
  public:
    
    /**
     * @brief Construct a new drivebase
     * 
     * @param leftMotors Motor group for left side of drivebase
     * @param rightMotors Motor group for right side of drivebase
     * @param robotBrain VEX brain object
     * @param inertialSensor VEX inertial sensor object
     * @param GPSSensor VEX gps sensor object
     */
    Drivebase(vex::motor_group& leftMotors, vex::motor_group& rightMotors, vex::brain& robotBrain, vex::inertial& inertialSensor, vex::gps& GPSSensor);

    /**
     * @brief Move the drivebase forwards or backwards a set number of inches
     * 
     * @param direction VEX directionType, forwards or backwards
     * @param inches Amount of inches to drive
     * @param velocityPercent Percent from 0 to 100
     */
    void drive(vex::directionType direction, double inches, int velocityPercent = 60);

    /**
     * @brief Turn the drivetrain quickly and precisely to a field-orientated angle
     * 
     * Uses a recursive PID algorithm to turn the robot to the desired angle.
     * Tuning is applied inside of drivebase.cpp.
     *
     * @param desiredAngle Angle in degrees to turn to
     * @param precision Amount of error in degrees that is accepted
     * @param secondsAllowed Maximum time to recursively turn
     * @param recursions Maximum number of individual attempts
     * @param minimumSpeed Lowest percentage speed the drivetrain will turn at
     */
    void turnTo(double desiredAngle, double precision = 0.5, double secondsAllowed = 2, int recursions = 5, double minimumSpeed = 1.8);

    /**
     * @brief Drive forwards or backwards quickly and precisely
     *
     * Uses a recursive PID algorithm to drive to a precise distance without requiring GPS strips
     * Tuning is applied inside of drivebase.cpp
     * 
     * @warning Has issues due to motor angle data not being precise enough--should be switched to tracking wheels
     * 
     * @param direction VEX directionType, forwards or backwards
     * @param desiredInches Amount of inches to drive
     * @param desiredAngle Field-orientated angle to maintain while driving
     * @param precision Amount of error in inches driven that is accepted
     * @param secondsAllowed Maximum time to recursively drive
     * @param recursions Maximum number of individual attempts
     * @param minSpeed Lowest percentage speed the drivetrain will move at
     */
    void driveTo(vex::directionType direction, double desiredInches, double desiredAngle, double precision = 0.5, double secondsAllowed = 10, int recursions = 5, double minSpeed = 1.0);

    /**
     * @brief Drive the robot to precise coordinates on the field
     * @warning CURRENTLY IN PROGRESS. DOES NOT WORK. 
     *
     * @param desiredX Desired X coordinate, in feet, field-orientated
     * @param desiredY Desired Y coordinate, in feet, field-orientated
     * @param precision Amount of error in inches driven that is accepted
     * @param secondsAllowed Maximum time to recursively drive
     * @param recursions Maximum number of individual attempts
     */
    void driveTo(double desiredX, double desiredY, double precision = 0.5, double secondsAllowed = 10, int recursions = 5);

    /**
     * @brief Returns the X coordinate of the robot center, in inches
     * 
     * @return double 
     */
    double getX() const;

    /**
     * @brief Returns the Y coordinate of the robot center, in inches
     * 
     * @return double 
     */
    double getY() const;

    /**
     * @brief Renders an approximation of the robot position on Brain screen
     * 
     */
    void renderRobot();

    /**
     * @brief Demo function for PID
     * 
     * Turns the robot to the setpoint but doesn't break
     * out of PID loop. Used as an example in interviews.
     * 
     * @param desiredAngle Angle to forever turn towards
     */
    void demoTo(double desiredAngle = 0);
};