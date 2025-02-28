#include "robot-config.h"
using namespace vex;

// Here, have a brain!
brain Brain;

// Controller
controller Controller1 = controller(primary);

// Drivetrain motors
motor leftBack = motor(PORT4, ratio6_1, true);
motor leftFront = motor(PORT5, ratio6_1, true);
motor leftTop = motor(PORT6, ratio6_1);
motor_group leftDrive = motor_group(leftBack, leftFront, leftTop);

motor rightBack = motor(PORT2, ratio6_1);
motor rightFront = motor(PORT1, ratio6_1);
motor rightTop = motor(PORT3, ratio6_1, true);
motor_group rightDrive = motor_group(rightBack, rightFront, rightTop);

// Intake motors
motor intakeUpper = motor(PORT12, ratio6_1, true);
motor intakeLower = motor(PORT7, ratio18_1, true); // 5.5W

// Wall Stake Mech
motor wallStakeMot = motor(PORT11, ratio18_1, true);

// Pneumatics
digital_out clampPneumatic = digital_out(Brain.ThreeWirePort.H);

// Other Sensors 175.1
gps GPS = gps(PORT15, 178.2); // Second number is degree offset from facing the front of the robot. It isn't 180 deg because the support is slightly bent :(
inertial Inertial = inertial(PORT13);
rotation wallStakeRot = rotation(PORT21, true);

// Drivetrain init
drivebase bot = drivebase(leftDrive, rightDrive, Brain, Inertial, GPS);

// wall stake init
wallStake ws = wallStake(wallStakeMot, wallStakeRot);

// Callback to stop intake
void stopIntake( void *arg ) {
  intakeLower.stop(coast);
  intakeUpper.stop(coast);
}

void vexcodeInit( void ) {}