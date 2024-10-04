using namespace vex;

extern brain Brain;
extern controller Controller1;


/* Swerve:

// Drivetrain motors
extern motor swerveFrontLeft1;
extern motor swerveFrontLeft2;
extern motor swerveFrontRight1;
extern motor swerveFrontRight2;
extern motor swerveBackRight1;
extern motor swerveBackRight2;
extern motor swerveBackLeft1;
extern motor swerveBackLeft2;

// Drivetrain sensors
extern rotation fL;
extern rotation fR;
extern rotation bR;
extern rotation bL;
*/

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

// Pneumatics
extern digital_out clampPneumatic;

// Other Sensors
extern gps GPS;
extern inertial Inertial;

// Thou shalt have a void
void  vexcodeInit( void );