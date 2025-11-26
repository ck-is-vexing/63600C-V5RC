#pragma once

#include "vex.h"
#include "control/drivebase.h"
#include "control/pneumatic.h"
#include "control/color-sensor.h"
#include "control/curves.h"
#include "control/rapid-trigger.h"

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

// Hopper motor
extern motor hopper;

// Pneumatics
extern Pneumatic redirect;
extern Pneumatic matchLoadMech;
extern Pneumatic wing;

// Sensors
extern gps GPS;
extern inertial imu;
extern ColorSensor intakeColor;

// Drivetrain
extern Drivebase bot;

// Rapid Trigger
extern RapidTrigger leftJoystick;
extern RapidTrigger rightJoystick;

void vexcodeInit( void );