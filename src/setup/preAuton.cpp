#include "setup/preAuton.h"

namespace preAuton {

  // Inertial sensor calibration function using GPS. This ends up being more precise than setting the robot up the same direction every time
  // For the robot to not average the angle at all, set seconds to 0.
  void inertialGPSCalibrate(double averageSeconds) {

    // Setup variables
    double averagedHeading = 0; // The final averaged heading
    int i; // A counter for the for loop. This must be defined outside of it for the final division.

    // For loop adds up all the different angles of the GPS. Step one of averaging the robot angle.
    for(i = 0; i < (averageSeconds * 25) + 1; i++){

      // Add the GPS heading to the total
      //averagedHeading += (GPS.heading() - gpsBlueAngle + 90);

      // Wait a short period because the GPS only updates a certain amount of times a second
      wait(40,msec);
    }

    // Divide averagedHeading by the amount of GPS headings added to it
    averagedHeading /= i;

    // Log the averaged heading versus the GPS heading
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Average: ");
    Controller1.Screen.print(averagedHeading);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("GPS: ");
    Controller1.Screen.print(GPS.heading());

    // Let the user know that the inertial sensor is now calibrating
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Calibrating...");

    // Set the inertial sensor's heading to the GPS heading.
    Inertial.setHeading(averagedHeading, deg);

    // Loop until the inertial sensor finishes setting the heading
    while(Inertial.isCalibrating()){

      //Wait to prevent wasted resources by iterating too fast
      wait(100,msec);
    }

    // Let the user know that calibration is complete
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Inertial Calibrated!");
  }

}