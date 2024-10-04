// This file contains functions that allow for all the inputs and current angle of the robot in a driver skills run to be recorded and then replayed in real-time

#include "vex.h"
#include <fstream>
using namespace vex; // Set the namespace to vex

// Define variables
int updateFrequency = 10; // One update every __ ms

// The length is calculated with this equation: (60 / (updateFrequency / 1000)) * 2
uint8_t leftDT[6000]; // Stores the velocities of the left side of the drivetrain
uint8_t rightDT[6000]; // Stores the velocities of the right side of the drivetrain
uint8_t angle[6000]; // Stores the angles of the robot using the inertial sensor


void saveTick(int t) {
  leftDT[t] = leftDrive.velocity(vex::percentUnits::pct) + 100;
  rightDT[t] = rightDrive.velocity(vex::percentUnits::pct) + 100;
}

void saveFinish() {
//https://www.programiz.com/cpp-programming/file-handling
}

void load(uint8_t data) {

}


void test() {

  // storage for some information to save
  uint8_t     myTestData[ 100 ];
  uint8_t     myReadBuffer[ 1000 ];
  
  // set the test data to something detectable
  for(int i=0;i<100;i++) {
    myTestData[i] = i * 2;
  }
  
  // write test data to SD Card
  int nWritten = Brain.SDcard.savefile( "test.h", myTestData, sizeof(myTestData) );

  // did that work ?
  if( nWritten > 0) {
    // display on screen how many bytes we wrote
    Brain.Screen.setCursor( 2, 2 );
    Brain.Screen.print( "We wrote %d bytes to the SD Card", nWritten );

    // now read it back into a different buffer
    int nRead = Brain.SDcard.loadfile( "test.h", myReadBuffer, sizeof(myReadBuffer) );

    // display on screen how many bytes we read
    Brain.Screen.setCursor( 3, 2 );
    Brain.Screen.print( "We read %d bytes from the SD Card", nRead );

    // and display some of the data
    Brain.Screen.setCursor( 6, 2 );
    for(int i=0;i<8;i++)
      Brain.Screen.print("%02X ", myReadBuffer);
    }

  else {
    Brain.Screen.printAt( 10, 40, "Error writing to the SD Card" );        
  }
}
