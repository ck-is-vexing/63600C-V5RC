#include "func/pose.h"

#include "vex.h"
#include "robot-config.h"
#include "definition.h"
#include "game/pre-auton.h"
#include <cmath>
#include <algorithm>

namespace {
  constexpr double X_OFFSET_GPS_INCHES            = -4.7;
  constexpr double Y_OFFSET_GPS_INCHES            = 8;


  constexpr double X_OFFSET_LEFT_DISTANCE_INCHES  = -5;
  constexpr double Y_OFFSET_LEFT_DISTANCE_INCHES  = 5;

  constexpr double X_OFFSET_FRONT_DISTANCE_INCHES = 5;
  constexpr double Y_OFFSET_FRONT_DISTANCE_INCHES = 4;

  constexpr double X_OFFSET_RIGHT_DISTANCE_INCHES = 5;
  constexpr double Y_OFFSET_RIGHT_DISTANCE_INCHES = 5;


  constexpr double PI_OVER_180                    = (M_PI / 180);


  pose::Pose odometryPose;
  bool odomRunning   = false;
  bool renderRunning = false;


  int odomTicker() {
    printl("Odom Ticker Thread Init");

    odometryPose           = pose::startingPose;
    double oldForwardAngle = 0;
    double oldSideAngle    = 0;

    odomForward.resetPosition();
    odomSide.resetPosition();

    while (true) {

      odometryPose.theta   =  fmod( (imu.heading() * PI_OVER_180), 2*M_PI );
      double triTheta      =  odometryPose.theta - M_PI;

      double forwardAngle  =  odomForward.position(vex::rotationUnits::deg);
      double sideAngle     =  odomSide.position(vex::rotationUnits::deg);

      double forwardIn     = (forwardAngle - oldForwardAngle) * PI_OVER_180;
      double sideIn        = (sideAngle    - oldSideAngle   ) * PI_OVER_180;

      
      double forwardDeltaX =  sin(triTheta)  * forwardIn;
      double sideDeltaX    =  cos(-triTheta) * sideIn;

      double forwardDeltaY =  cos(triTheta)  * forwardIn;
      double sideDeltaY    =  sin(-triTheta) * sideIn;
      

      odometryPose.x      += (forwardDeltaX  + sideDeltaX);
      odometryPose.y      += (forwardDeltaY  + sideDeltaY);


      if (!odomRunning) { break; }

      oldForwardAngle      =  forwardAngle;
      oldSideAngle         =  sideAngle;

      wait (10, msec);
    }

    printl("Thread Exiting!");
    return 0;
  }

  
  int renderTicker() {
    printl("Render Ticker Thread Init");

    while (true) {

      pose::render::renderRobot();
      //printl(odometryPose.x << "    " << odometryPose.y << "      " << odometryPose.theta);
      if (!renderRunning) { break; }

      wait(100, msec);
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

  robotPose.theta = fmod( (imu.heading() * PI_OVER_180), 2*M_PI );

  double triAngle = M_PI - atan(X_OFFSET_GPS_INCHES / Y_OFFSET_GPS_INCHES) - robotPose.theta;
  double triHyp   = sqrt((X_OFFSET_GPS_INCHES * X_OFFSET_GPS_INCHES) + (Y_OFFSET_GPS_INCHES * Y_OFFSET_GPS_INCHES));

  double offsetX  = sin(triAngle) * triHyp;
  double offsetY  = cos(triAngle) * triHyp;
  
  if (preAuton::startingGPS.flip) {
    robotPose.y   = (GPS.xPosition(distanceUnits::in) * preAuton::startingGPS.x + offsetX);
    robotPose.x   = (GPS.yPosition(distanceUnits::in) * preAuton::startingGPS.y + offsetY);
  } else {
    robotPose.x   = (GPS.xPosition(distanceUnits::in) * preAuton::startingGPS.x + offsetX);
    robotPose.y   = (GPS.yPosition(distanceUnits::in) * preAuton::startingGPS.y + offsetY);
  }

  return robotPose;
}

pose::Pose pose::calcPoseDist() {
  pose::Pose robotPose;

  robotPose.theta = fmod( (imu.heading() * PI_OVER_180), 2*M_PI );

  double frontVal = frontDist.objectDistance(vex::distanceUnits::in);
  double sideVal;

  double sideOffsetX;
  double sideOffsetY;

  if        (leftDist.isObjectDetected()) {
    sideVal       = leftDist.objectDistance(vex::distanceUnits::in);
    sideOffsetX   = X_OFFSET_LEFT_DISTANCE_INCHES;
    sideOffsetY   = Y_OFFSET_LEFT_DISTANCE_INCHES;

  } else if (rightDist.isObjectDetected()) {
    sideVal       = rightDist.objectDistance(vex::distanceUnits::in);
    sideOffsetX   = X_OFFSET_RIGHT_DISTANCE_INCHES;
    sideOffsetY   = Y_OFFSET_RIGHT_DISTANCE_INCHES;

  } else {
    printl("No Object Detected!");
  }


  double distX;
  double distY;
  double triAngleX;
  double triAngleY;
  double triHypX;
  double triHypY;

  double sinTheta = sin(robotPose.theta);
  
  if (((robotPose.theta > M_PI/4) && (robotPose.theta < 3*M_PI/4)) || ((robotPose.theta > 5*M_PI/4) && (robotPose.theta < 7*M_PI/4))) {
    distX         = frontVal * sinTheta;
    distY         = sideVal  * sinTheta;

    triAngleX     = M_PI - atan(X_OFFSET_FRONT_DISTANCE_INCHES / Y_OFFSET_FRONT_DISTANCE_INCHES) - robotPose.theta;
    triHypX       = sqrt((X_OFFSET_FRONT_DISTANCE_INCHES * X_OFFSET_FRONT_DISTANCE_INCHES) + (Y_OFFSET_FRONT_DISTANCE_INCHES * Y_OFFSET_FRONT_DISTANCE_INCHES));

    triAngleY     = M_PI - atan(sideOffsetX / sideOffsetY) - robotPose.theta;
    triHypY       = sqrt((sideOffsetX * sideOffsetX) + (sideOffsetY * sideOffsetY));

  } else {
    distX         = sideVal  * sinTheta;
    distY         = frontVal * sinTheta;

    triAngleX     = M_PI - atan(sideOffsetX / sideOffsetY) - robotPose.theta;
    triHypX       = sqrt((sideOffsetX * sideOffsetX) + (sideOffsetY * sideOffsetY));

    triAngleY     = M_PI - atan(X_OFFSET_FRONT_DISTANCE_INCHES / Y_OFFSET_FRONT_DISTANCE_INCHES) - robotPose.theta;
    triHypY       = sqrt((X_OFFSET_FRONT_DISTANCE_INCHES * X_OFFSET_FRONT_DISTANCE_INCHES) + (Y_OFFSET_FRONT_DISTANCE_INCHES * Y_OFFSET_FRONT_DISTANCE_INCHES));
  }


  robotPose.x     = sin(triAngleX) * triHypX;
  robotPose.y     = cos(triAngleY) * triHypY;
  
  robotPose.x    -= 72; // Adjust coordinate center to be the middle of field
  robotPose.y    -= 72;

  printl(distX << "    " << distY << "      " << robotPose.theta << "    " << robotPose.x << "    " << robotPose.y);

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


void pose::render::initTicker() {
  renderRunning = true;
  thread renderTickerThread = thread(renderTicker);
}

void pose::render::killTicker() {
  renderRunning = false;
}

void pose::render::renderRobot() {

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
  pose::Pose pose_dist = calcPoseDist();

  double x_gps         = pose_gps.x  * 5/3; // Convert inches to pixels
  double y_gps         = pose_gps.y  * 5/3;
  double x2_gps        = x_gps  + sin(pose_gps.theta - M_PI)  * VECTOR_LENGTH_PIXELS;
  double y2_gps        = y_gps  + cos(pose_gps.theta - M_PI)  * VECTOR_LENGTH_PIXELS;

  double x_odom        = pose_odom.x * 5/3;
  double y_odom        = pose_odom.y * 5/3;
  double x2_odom       = x_odom + sin(pose_odom.theta - M_PI) * VECTOR_LENGTH_PIXELS;
  double y2_odom       = y_odom + cos(pose_odom.theta - M_PI) * VECTOR_LENGTH_PIXELS;

  double x_dist        = pose_dist.x * 5/3;
  double y_dist        = pose_dist.y * 5/3;
  double x2_dist       = x_dist + sin(pose_dist.theta - M_PI) * VECTOR_LENGTH_PIXELS;
  double y2_dist       = y_dist + cos(pose_dist.theta - M_PI) * VECTOR_LENGTH_PIXELS;

  Brain.Screen.setPenWidth(6);

  Brain.Screen.setPenColor(blue);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawCircle(x_gps + 120, -y_gps + 120, 8);
  Brain.Screen.drawLine(x_gps + 120, -y_gps + 120, x2_gps + 120, -y2_gps + 120);

  Brain.Screen.setPenColor(red);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawCircle(x_odom + 120, -y_odom + 120, 8);
  Brain.Screen.drawLine(x_odom + 120, -y_odom + 120, x2_odom + 120, -y2_odom + 120);

  Brain.Screen.setPenColor(green);
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawCircle(x_dist + 120, -y_dist + 120, 8);
  Brain.Screen.drawLine(x_dist + 120, -y_dist + 120, x2_dist + 120, -y2_dist + 120);

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