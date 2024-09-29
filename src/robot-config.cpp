#include "vex.h"
#include "Functions/swerveControl.cpp" // Drivetrain control

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// Here, have a brain!
brain Brain;

// Controller
controller Controller1 = controller(primary);

/* Swerve
// Drivetrain motors
motor swerveFrontLeft1 = motor(PORT1, ratio18_1);
motor swerveFrontLeft2 = motor(PORT2, ratio18_1, true);
motor swerveFrontRight1 = motor(PORT3, ratio18_1);
motor swerveFrontRight2 = motor(PORT4, ratio18_1, true);
motor swerveBackLeft1 = motor(PORT5, ratio18_1);
motor swerveBackLeft2 = motor(PORT6, ratio18_1, true);
motor swerveBackRight1 = motor(PORT7, ratio18_1);
motor swerveBackRight2 = motor(PORT8, ratio18_1, true);

// Drivetrain sensors
rotation fL = rotation(PORT9);
rotation fR = rotation(PORT10);
rotation bR = rotation(PORT11);
rotation bL = rotation(PORT12);

// Initiate drivetrain
SwerveDrivetrain Drivetrain(swerveFrontLeft1,swerveFrontLeft2,swerveFrontRight1,swerveFrontRight2,swerveBackRight1,swerveBackRight2,swerveBackLeft1,swerveBackLeft2,fL,fR,bR,bL);
*/

// Drivtrain motors
motor leftBack = motor(PORT13, ratio6_1, true);
motor leftFront = motor(PORT12, ratio6_1, true);
motor leftTop = motor(PORT11, ratio6_1);
motor_group leftDrive = motor_group(leftBack, leftFront, leftTop);

motor rightBack = motor(PORT18, ratio6_1);
motor rightFront = motor(PORT19, ratio6_1);
motor rightTop = motor(PORT20, ratio6_1, true);
motor_group rightDrive = motor_group(rightBack, rightFront, rightTop);

// Other Sensors
gps GPS = gps(PORT1);
inertial Inertial = inertial(PORT2);


void vexcodeInit( void ) {
  // nothing to initialize because we are soooooo boring around here and i have been coding for like 13 hours and why am i still here just to suffer and this comment will probably get deleted anyways so there is no reason for me to not randomly start ranting about code
}