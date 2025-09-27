#pragma once

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
extern motor intakeBack;

// Pneumatics
extern digital_out intakePneumatic;

// Other Sensors
extern gps GPS;
extern inertial Inertial;
extern rotation wallStakeRot;

// Drivetrain
extern Drivebase bot;

// Rapid Trigger
extern RapidTrigger leftJoystick;
extern RapidTrigger rightJoystick;

void stopIntake( void *arg );

void vexcodeInit( void );