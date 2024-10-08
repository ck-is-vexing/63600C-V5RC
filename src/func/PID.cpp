#include "vex.h" // Include VEX headers

// This class contains a general PID implementation to be used for more specific applications, such as turnTo
class PID {
  private:
    double Kp; // Proportional Tuning
    double Ki; // Integral Tuning
    double Kd; // Derivative Tuning
    double integral = 0; // Current integral value
    double derivative; // Current derivative value
    int dT; // The change in time, in ms, of every tick
    double oldError = 0; // The previous error, used for calculating Integral
  public:

    // Proportional, integral, and derivative tuning respectively
    // deltaTime should be the change in time, in ms, between ticks
    PID(double Kproportional, double Kintegral, double Kderivative, int deltaTime){
      Kp = Kproportional;
      Ki = Kintegral;
      Kd = Kderivative;
      dT = deltaTime;
    }

    // Returns the PID value for one tick
    // setpoint is the desired value for the variable to approach
    // pv is the current value of the variable
    double update(double setpoint, double pv){ 

      // Update error
      // It's the difference between what is desired and what the current value is.
      double error = setpoint - pv;

      // Update integral
      // The integral calculates the area in between the graph of pv and the x-axis
      // To calculate integral, every tick the 
      integral += error * dT;

      // Update Derivative
      // Because the code only executes so often, the equation essentially is (y2 - y1) / (x2 - x1)
      // The y values are the current error, and the previous error
      // Because the change is always one tick, the denomenator only has to be the length of one tick
      derivative = (error - oldError) / dT;

      // Update old error
      oldError = error;

      // Calculate the return value
      return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

    void reset(){

      // Reset variables
      integral = 0;

    }

    // Runs a general instance of the PID
    bool run(double setpoint, double precision = 1, double secondsAllowed = 2, int recursions = 5){
      
      // vars
      double current;

      // Loop of recursions
      for (int i = 0; i < recursions; i++){

        // Loop of ticks, converting seconds allowed into sets of dT ms
        for (int t = 0; t < (secondsAllowed * 1000 / dT); t++){
          
          current = 0; // you know
          update(setpoint, current);

          //Spin motors
          // ...

          //Check for completion of the loop of ticks
          if (current < (setpoint + precision) && current > (setpoint - precision)){
            
            //Break the tick loop
            break;
          }

          //Prevent wasted resources by waiting a short period of time before iterating
          wait(dT,msec); 
        }

        //Stop motors
        // ...

        //Reset integral
        integral = 0;

        //Wait a little bit in case the system is still in motion
        wait(200,msec);

        //Update currentAngle
        current = 0;

        //Check if desired angle was achieved
        if (current < (setpoint + precision) && current > (setpoint - precision)){

          //Set the motors to stop on coast
          // ...

          //Break the recursion loop and the whole turnTo, because it has reached the desired angle
          break;
        }  

        //If the correct angle was not achieved, the code will recurse
      }
      return true; // returns true because the process has ended
    }
};