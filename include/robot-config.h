#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"
#include "control/drivebase.h"
#include "control/ccurve.h"
using namespace vex;

extern brain Brain;
extern controller Controller1;

// Drivetrain motors
extern motor leftBack;
extern motor leftMiddle;
extern motor leftFront;
extern motor_group leftDrive;

extern motor rightBack;
extern motor rightMiddle;
extern motor rightFront;
extern motor_group rightDrive;


// Intake motors
extern motor intakeUpper;
extern motor intakeLower;

// Pneumatics
extern digital_out intakePneumatic;

// Other Sensors
extern gps GPS;
extern inertial Inertial;
extern rotation wallStakeRot;

// Drivetrain
extern drivebase bot;

// Rapid Trigger
extern RapidTrigger leftJoystick;
extern RapidTrigger rightJoystick;

// Callback to stop intake
void stopIntake( void *arg );

// Initializer for robot configuration
void vexcodeInit( void );

#endif