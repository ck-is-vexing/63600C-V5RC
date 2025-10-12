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
motor intakeUpper = motor(PORT9, ratio6_1); // 5.5W
motor intakeLower = motor(PORT10, ratio18_1); // 5.5W
motor intakeBack = motor(PORT1, ratio18_1); //  5.5W

// Pneumatics
digital_out intakePneumatic = digital_out(Brain.ThreeWirePort.H);

// Other Sensors 175.1
gps GPS = gps(PORT15, 178.2); // Second number is degree offset from facing the front of the robot. It isn't 180 deg because the support is slightly bent :(
inertial Inertial = inertial(PORT13);
rotation wallStakeRot = rotation(PORT21, true);

// 6m Drivetrain init
Drivebase bot = Drivebase(leftDrive, rightDrive, Brain, Inertial, GPS);

// RT init
RapidTrigger leftJoystick = RapidTrigger(Controller1.Axis3, curves::linear);
RapidTrigger rightJoystick = RapidTrigger(Controller1.Axis2, curves::linear);

void stopIntake( void *arg ) {
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
  intakeBack.stop(coast);
}

void vexcodeInit( void ) {}