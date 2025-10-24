#include "robot-config.h"
using namespace vex;

brain Brain;
controller Controller1 = controller(primary);

// Drivetrain motors
motor leftBack = motor(PORT13, ratio6_1, true);
motor leftMiddle = motor(PORT19, ratio6_1, true);
motor leftFront = motor(PORT12, ratio6_1, true);
motor_group leftDrive = motor_group(leftBack, leftMiddle, leftFront);

motor rightBack = motor(PORT20, ratio6_1);
motor rightMiddle = motor(PORT14, ratio6_1);
motor rightFront = motor(PORT11, ratio6_1);
motor_group rightDrive = motor_group(rightBack, rightMiddle, rightFront);

// Intake motors
motor intakeUpper = motor(PORT3, ratio6_1, true); // 5.5W
motor intakeLower = motor(PORT1, ratio18_1); // 5.5W
motor intakeBack = motor(PORT4, ratio18_1, true); //  5.5W

// Hopper motor
motor hopper = motor(PORT2, ratio18_1, true); // 5.5W

// Pneumatics
Pneumatic redirect = Pneumatic(Brain.ThreeWirePort.A, true);
Pneumatic matchLoadMech = Pneumatic(Brain.ThreeWirePort.B, true);

// Other Sensors
gps GPS = gps(PORT7, 359);
inertial Inertial = inertial(PORT6);

// 6m Drivetrain init
Drivebase bot = Drivebase(leftDrive, rightDrive, Brain, Inertial, GPS);

// RT init
RapidTrigger leftJoystick = RapidTrigger(Controller1.Axis3, curves::linear);
RapidTrigger rightJoystick = RapidTrigger(Controller1.Axis2, curves::linear);

void vexcodeInit( void ) {}