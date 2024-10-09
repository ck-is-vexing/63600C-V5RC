#include "vex.h"
//#include "func/swerveControl.cpp" // Drivetrain control


using namespace vex;
//using signature = vision::signature;
//using code = vision::code;

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

// Drivetrain motors
motor leftBack = motor(PORT9, ratio6_1, true);
motor leftFront = motor(PORT10, ratio6_1, true);
motor leftTop = motor(PORT7, ratio6_1);
motor_group leftDrive = motor_group(leftBack, leftFront, leftTop);

motor rightBack = motor(PORT12, ratio6_1);
motor rightFront = motor(PORT11, ratio6_1);
motor rightTop = motor(PORT13, ratio6_1, true);
motor_group rightDrive = motor_group(rightBack, rightFront, rightTop);

// Intake motors
motor intakeUpper = motor(PORT20, ratio18_1, true);
motor intakeLower = motor(PORT19, ratio18_1, true); // 5.5W

// Pneumatics
digital_out clampPneumatic = digital_out(Brain.ThreeWirePort.A);

// Other Sensors
gps GPS = gps(PORT15);
inertial Inertial = inertial(PORT6);


void vexcodeInit( void ) {
  // sí
}