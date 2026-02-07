#include "control/pose.h"

#include "vex.h"
#include "robot-config.h"
#include "definition.h"
#include "game/pre-auton.h"
#include <cmath>
#include <algorithm>

namespace {
  constexpr double X_OFFSET_GPS_INCHES = -4.7;
  constexpr double Y_OFFSET_GPS_INCHES = 8;

  constexpr double PI_OVER_180         = (M_PI / 180);

  pose::Pose odometryPose;
  bool odomRunning = false;

  int odomTicker() {
    printl("Odom Ticker Thread Init");

    odometryPose           = pose::startingPose;
    double oldForwardAngle = 0;
    double oldSideAngle    = 0;

    odomForward.resetPosition();
    odomSide.resetPosition();

    while (true) {

      // TODO: Subtract the delta turn arc distance every tick so it adjusts for the robot turning? this is just the angle in rad.

      odometryPose.theta   = imu.heading() * PI_OVER_180;
      double triTheta      = odometryPose.theta - M_PI;

      double forwardIn     = (odomForward.position(vex::rotationUnits::deg) - oldForwardAngle) * PI_OVER_180;
      double sideIn        = (odomSide.position(vex::rotationUnits::deg) - oldSideAngle) * PI_OVER_180;

      oldForwardAngle      =  odomForward.position(vex::rotationUnits::deg);
      oldSideAngle         =  odomSide.position(vex::rotationUnits::deg);

      double forwardDeltaX = sin(triTheta)  * forwardIn;
      double sideDeltaX    = cos(-triTheta) * sideIn;

      double forwardDeltaY = cos(triTheta)  * forwardIn;
      double sideDeltaY    = sin(-triTheta) * sideIn;
      
      odometryPose.x      += (forwardDeltaX + sideDeltaX);
      odometryPose.y      += (forwardDeltaY + sideDeltaY);

      if (!odomRunning) { break; }

      wait (10, msec);
    }

    printl("Thread Exiting!");
    return 0;
  }
}


pose::Pose::Pose()
: x(0.0), y(0.0), theta(0.0) {}

pose::Pose::Pose(double _x, double _y, double _theta)
: x(_x), y(_y), theta(_theta) {}


pose::Pose pose::startingPose;


pose::Pose pose::calcPoseGPS() {
  pose::Pose robotPose;

  robotPose.theta = imu.heading() * PI_OVER_180;

  double triAngle = M_PI - atan(X_OFFSET_GPS_INCHES / Y_OFFSET_GPS_INCHES) - robotPose.theta;
  double triHyp   = sqrt((X_OFFSET_GPS_INCHES * X_OFFSET_GPS_INCHES) + (Y_OFFSET_GPS_INCHES * Y_OFFSET_GPS_INCHES));

  double offsetX  = sin(triAngle) * triHyp;
  double offsetY  = cos(triAngle) * triHyp;
  
  if (preAuton::startingGPS.flip) {
    robotPose.y     = (GPS.xPosition(distanceUnits::in) * preAuton::startingGPS.x + offsetX);
    robotPose.x     = (GPS.yPosition(distanceUnits::in) * preAuton::startingGPS.y + offsetY);
  } else {
    robotPose.x     = (GPS.xPosition(distanceUnits::in) * preAuton::startingGPS.x + offsetX);
    robotPose.y     = (GPS.yPosition(distanceUnits::in) * preAuton::startingGPS.y + offsetY);
  }

  return robotPose;
}


const pose::Pose pose::odom::getPose() {
  return odometryPose;
}

void pose::odom::initTicker() {
  odomRunning = true;
  thread odomTickerThread = thread(odomTicker);
}

void pose::odom::killTicker() {
  odomRunning = false;
}


void pose::renderRobot() {

  // VEX Brain is 480x240p
  // 20p = 1 foot

  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(white);
  Brain.Screen.setPenWidth(2);

  Brain.Screen.setFillColor(yellow);
  Brain.Screen.drawCircle(120,120,8);

  // Draw field
  for(int x = 0; x <= 6; x++){
    Brain.Screen.drawLine(x * 40, 0, x * 40, 240);
  }
  for(int y = 0; y <= 6; y++){
    Brain.Screen.drawLine(0, y * 40, 240, y * 40);
  }

  // Draw robot vector
  constexpr int VECTOR_LENGTH_PIXELS = 40;

  pose::Pose pose_gps  = calcPoseGPS();
  pose::Pose pose_odom = odom::getPose();

  double x_gps   = pose_gps.x  * 5/3; // Convert inches to pixels
  double y_gps   = pose_gps.y  * 5/3;
  double x2_gps  = x_gps  + sin(pose_gps.theta - M_PI)  * VECTOR_LENGTH_PIXELS;
  double y2_gps  = y_gps  + cos(pose_gps.theta - M_PI)  * VECTOR_LENGTH_PIXELS;

  double x_odom  = pose_odom.x * 5/3;
  double y_odom  = pose_odom.y * 5/3;
  double x2_odom = x_odom + sin(pose_odom.theta - M_PI) * VECTOR_LENGTH_PIXELS;
  double y2_odom = y_odom + cos(pose_odom.theta - M_PI) * VECTOR_LENGTH_PIXELS;

  Brain.Screen.setPenWidth(6);

  Brain.Screen.setPenColor(blue);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawCircle(x_gps + 120, -y_gps + 120, 8);
  Brain.Screen.drawLine(x_gps + 120, -y_gps + 120, x2_gps + 120, -y2_gps + 120);

  Brain.Screen.setPenColor(red);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawCircle(x_odom + 120, -y_odom + 120, 8);
  Brain.Screen.drawLine(x_odom + 120, -y_odom + 120, x2_odom + 120, -y2_odom + 120);

  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenWidth(1);
  
  Brain.Screen.setCursor(4, 30);
  Brain.Screen.print("Angle: ");
  Brain.Screen.print(pose_gps.theta);

  Brain.Screen.setCursor(5, 30);
  Brain.Screen.print("gpsX: ");
  Brain.Screen.print(pose_gps.x);
  Brain.Screen.setCursor(6, 30);
  Brain.Screen.print("gpsY: ");
  Brain.Screen.print(pose_gps.y);

  Brain.Screen.setCursor(7, 30);
  Brain.Screen.print("odomX: ");
  Brain.Screen.print(pose_odom.x);
  Brain.Screen.setCursor(8, 30);
  Brain.Screen.print("odomY: ");
  Brain.Screen.print(pose_odom.y);
}