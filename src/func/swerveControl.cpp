#include "vex.h" // Include the vex headers
#include "PID.cpp" // Include pid class
using namespace vex; // Set the namespace to vex

class SwerveWheel {
  private:
    motor mot1 = motor(PORT1, ratio18_1);
    motor mot2 = motor(PORT1, ratio18_1);
    rotation rot = rotation(PORT1);

    double mot1Vel = 0;
    directionType mot1Dir = fwd;

    double mot2Vel = 0;
    directionType mot2Dir = fwd;

    PID turnToPID = PID(1,1,1,10);
  public:
    // Motors and sensor for this specific wheel
    SwerveWheel(motor motor1, motor motor2, rotation rotation){
      mot1 = motor1;
      mot2 = motor2;
      rot = rotation;
    }

    // Spins the wheel
    void spin(directionType dir, double vel = 100){ // Velocity in %
      mot1Dir = dir;
      mot1Vel = vel;
      mot2Dir = dir;
      mot2Vel = vel;
    }

    // Brakes the wheel with a certain braking type
    void brake(brakeType type){
      mot1Vel = 0;
      mot2Vel = 0;
      mot1.stop(type);
      mot2.stop(type);
    }
    
    void turnTo(double desiredAngle, double precision = 0.3, double secondsAllowed = 2, int recursions = 5){

    }

    // PID function to aim the wheel housing to a desired angle relative to the robot
    void setAngle(double desiredAngle, double precision = 0.3, double secondsAllowed = 2, int recursions = 5){
          
      /*  
        desiredAngle: The angle that the programmer desires the wheel housing(s) to turn to
        precision: How close the wheel housing must be to the desired angle for it to quit the turn.
        secondsAllowed: A cap for how long the code is allowed to repeat for each recurson
        recursions: The amount of seperate times the robot should be allowed to pass desiredAngle and continue to turn towards it.
      */

      // Tuning
      double Kp = 1;
      double Ki = 1;
      double Kd = 1;

      // Variables
      double currentAngle = 0; // The current angle of the wheel housing, as a variable
      double error = 0; // How far the wheel housing is from the intended angle, calculated with the difference of desiredAngle and currentAngle.
      double motorVelocity = 0; // The velocity the drivetrain motors are set to
      double integral = 0; //The integral parameter

      // Force the desired angle to be within 360 degrees by using mod
      desiredAngle = fmod(desiredAngle,360); 

      // Loop of recursions
      for (int i = 0; i < recursions; i++){

        // Loop of ticks, converting seconds allowed into sets of 10 ms
        for (int t = 0; t < (secondsAllowed * 100); t++){
          
          // Update currentAngle
          currentAngle = Inertial.heading();

          //How far the robot is from the intended angle, calculated with the difference of desiredAngle and currentAngle.
          error = desiredAngle - currentAngle; 

          //Find the shortest possible route to reach target angle
          if (error > 180){
            error -= 360;
          } else if (error < -180){
            error += 360;
          }
          
          //Update Integral
          integral += (error * t / 1000); //Integral = Integral + (current error) * (tick number, one every 10 milliseconds) / (1000, to convert to seconds)

          //Calculating the final velocity for motors
          motorVelocity = (Kp * error) + (Ki * integral); //The first set of parenthesis contains Proportional control; the second Integral control. Each is multiplied by a tuning variable.

          //Spin motors
          // ...

          //Check for completion of the loop of ticks
          if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
            (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) || //Check for completion at a near 0 or 360 degree angle
            (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){ //Same as above
            
            //Break the tick loop
            break;
          }

          //Prevent wasted resources by waiting a short period of time before iterating
          wait(10,msec); 
        }

        //Stop motors
        // ...

        //Reset integral
        integral = 0;

        //Wait a little bit in case the robot is still in motion
        wait(200,msec);

        //Update currentAngle
        currentAngle = Inertial.heading();

        //Check if desired angle was achieved
        if ((currentAngle < (desiredAngle + precision) && currentAngle > (desiredAngle - precision)) ||
          (currentAngle < (desiredAngle + precision - 360) && currentAngle > (desiredAngle - precision - 360)) || //Check for completion at a near 0 or 360 degree angle
          (currentAngle < (desiredAngle + precision + 360) && currentAngle > (desiredAngle - precision + 360)) ){ //Same as above

          //Update controller screen to tell user the robot has finished the turn.
          Controller1.Screen.clearScreen(); //Clear the scren
          Controller1.Screen.setCursor(1, 1); //Set the cursor to the first line of the controller
          Controller1.Screen.print("Turn Complete!"); //Print a nice message
          Controller1.Screen.setCursor(2, 1); //Set the cursor to the second line
          Controller1.Screen.print("Angle: "); //Print a label for currentAngle
          Controller1.Screen.print(currentAngle); //Print currentAngle to show if the code encountered a bug

          //Set the motors to stop on coast
          // ...

          //Break the recursion loop and the whole turnTo, because it has reached the desired angle
          break;
        }  

        //If the correct angle was not achieved, the code will recurse
      }
    }

    // Main call to update motor speed based on everything else--this needs to be seperate for all the fancy stuff to happen (like turning while driving)
    // This MUST be called every time the drivetrain needs to update
    void tick(){
      if (mot1Vel > 0 || mot2Vel > 0){
        mot1.spin(mot1Dir, mot1Vel, velocityUnits::pct);
        mot2.spin(mot2Dir, mot2Vel, velocityUnits::pct);
      }
    }
};

class SwerveDrivetrain {
  private:
    // Defining vars
    motor fl1 = motor(PORT1, ratio18_1);
    motor fl2 = motor(PORT1, ratio18_1);

    motor fr1 = motor(PORT1, ratio18_1);
    motor fr2 = motor(PORT1, ratio18_1);

    motor br1 = motor(PORT1, ratio18_1);
    motor br2 = motor(PORT1, ratio18_1);

    motor bl1 = motor(PORT1, ratio18_1);
    motor bl2 = motor(PORT1, ratio18_1);

    rotation fl = rotation(PORT1);
    rotation fr = rotation(PORT1);
    rotation br = rotation(PORT1);
    rotation bl = rotation(PORT1);
  public:
    // Drivetrain parameters
    SwerveDrivetrain(motor frontLeft1, motor frontLeft2, motor frontRight1, motor frontRight2, motor backRight1, motor backRight2, motor backLeft1, motor backLeft2, rotation frontLeft, rotation frontRight, rotation backRight, rotation backLeft){
      // Motors
      fl1 = frontLeft1;
      fl2 = frontLeft2;

      fr1 = frontRight1;
      fr2 = frontRight2;

      br1 = backRight1;
      br2 = backRight2;

      bl1 = backLeft1;
      bl2 = backLeft2;

      // Sensors
      fl = frontLeft;
      fr = frontRight;
      br = backRight;
      bl = backLeft;
    }

    // Basic drive function which assumes the wheels are aligned
    void drive(directionType dir, double dist, double vel = 100){ // Distance in inches, velocity in percent
      // Conversion from inches to degrees of rotation for the motors
      double rot = dist / 0.00799918981481; //(Circumphrence of the wheels) / (The gear ratio of the robot) / (360, to put the number into degrees)

      // Spin motors
      fl1.spinFor(dir, rot, rotationUnits::deg, vel, velocityUnits::pct, false);
      fl2.spinFor(dir, -rot, rotationUnits::deg, vel, velocityUnits::pct, false);

      fr1.spinFor(dir, rot, rotationUnits::deg, vel, velocityUnits::pct, false);
      fr2.spinFor(dir, -rot, rotationUnits::deg, vel, velocityUnits::pct, false);

      br1.spinFor(dir, rot, rotationUnits::deg, vel, velocityUnits::pct, false);
      br2.spinFor(dir, -rot, rotationUnits::deg, vel, velocityUnits::pct, false);

      bl1.spinFor(dir, rot, rotationUnits::deg, vel, velocityUnits::pct, false);
      bl2.spinFor(dir, -rot, rotationUnits::deg, vel, velocityUnits::pct, true);
    }
};