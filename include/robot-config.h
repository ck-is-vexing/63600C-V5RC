#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"
#include "control/drivebase.h"
using namespace vex;

extern brain Brain;
extern controller Controller1;

// Drivetrain motors
extern motor leftBack;
extern motor leftFront;
extern motor leftTop;
extern motor_group leftDrive;

extern motor rightBack;
extern motor rightFront;
extern motor rightTop;
extern motor_group rightDrive;


// Intake motors
extern motor intakeUpper;
extern motor intakeLower;

// Wall Stake Motor
extern motor wallStake;

// Pneumatics
extern digital_out clampPneumatic;

// Other Sensors
extern gps GPS;
extern inertial Inertial;

// Drivetrain
extern drivebase bot;

// Callback to stop intake
void stopIntake( void *arg );

// Initializer for robot configuration
void vexcodeInit( void );

#endif