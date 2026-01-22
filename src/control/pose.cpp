#include "control/pose.h"

#include "vex.h"
#include "robot-config.h"
#include <cmath>

namespace {
  constexpr double X_OFFSET_GPS_INCHES = 2;
  constexpr double Y_OFFSET_GPS_INCHES = -5.5;
}

pose::Pose::Pose()
: x(0.0), y(0.0), theta(0.0) {}

pose::Pose::Pose(double _x, double _y, double _theta)
: x(_x), y(_y), theta(_theta) {}

pose::Pose pose::getPoseGPS() {
  pose::Pose robotPose;

  robotPose.theta = (imu.heading() * M_PI / 180);

  double angle = 180 - atan(X_OFFSET_GPS_INCHES / Y_OFFSET_GPS_INCHES) - robotPose.theta;

  double deltaX = cos(angle) * sqrt((X_OFFSET_GPS_INCHES * X_OFFSET_GPS_INCHES) + (Y_OFFSET_GPS_INCHES * Y_OFFSET_GPS_INCHES));
  double deltaY = sin(angle) * sqrt((X_OFFSET_GPS_INCHES * X_OFFSET_GPS_INCHES) + (Y_OFFSET_GPS_INCHES * Y_OFFSET_GPS_INCHES));
  
  robotPose.x = (GPS.xPosition(distanceUnits::in) + deltaX);
  robotPose.y = (GPS.yPosition(distanceUnits::in) + deltaY);

  return robotPose;
}

pose::Pose pose::getPoseOdom() {
  pose::Pose robotPose;

  robotPose.theta = (imu.heading() * M_PI / 180);

  return robotPose;
}

void pose::renderRobot() {

  // VEX Brain is 480x240p
  // 20p = 1 foot
  // GPS sensor (0,0) is at the center of the field

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

  pose::Pose pose_gps  = getPoseGPS();
  pose::Pose pose_odom = getPoseOdom();

  double x_gps   = pose_gps.x  * 5/3; // Convert inches to pixels
  double y_gps   = pose_gps.y  * 5/3;
  double x2_gps  = x_gps  + cos(pose_gps.theta)  * VECTOR_LENGTH_PIXELS;
  double y2_gps  = y_gps  + sin(pose_gps.theta)  * VECTOR_LENGTH_PIXELS;

  double x_odom  = pose_odom.x * 5/3;
  double y_odom  = pose_odom.y * 5/3;
  double x2_odom = x_odom + cos(pose_odom.theta) * VECTOR_LENGTH_PIXELS;
  double y2_odom = y_odom + sin(pose_odom.theta) * VECTOR_LENGTH_PIXELS;

  Brain.Screen.setPenWidth(6);

  Brain.Screen.setPenColor(blue);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawCircle(x_gps, y_gps, 8);
  Brain.Screen.drawLine(x_gps + 120, y_gps + 120, x2_gps + 120, y2_gps + 120);

  Brain.Screen.setPenColor(red);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawCircle(x_odom + 120, y_odom + 120, 8);
  Brain.Screen.drawLine(x_odom + 120, y_odom + 120, x2_odom + 120, y2_odom + 120);

  Brain.Screen.setCursor(4, 30);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenWidth(1);
  Brain.Screen.print("Angle: ");
  Brain.Screen.print(pose_gps.theta);
  Brain.Screen.setCursor(5, 30);
  Brain.Screen.print("X: ");
  Brain.Screen.print(pose_gps.x);
  Brain.Screen.setCursor(6, 30);
  Brain.Screen.print("Y: ");
  Brain.Screen.print(pose_gps.y);
}