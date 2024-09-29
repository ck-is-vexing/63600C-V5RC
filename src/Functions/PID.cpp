#include "vex.h" // Include the vex headers
using namespace vex; // Set the namespace to vex

class PID {
  private:
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double derivative;
    int t;
    double oldError;
  public:
    PID(double Kproportional, double Kintegral, double Kderivative){
      Kp = Kproportional;
      Ki = Kintegral;
      Kd = Kderivative;
    }

    double update(double setpoint, double pv){ 
      // Update error
      double error = setpoint - pv;

      // Update Integral
      integral += (error * t);

      // Update Derivative
      derivative = (error - oldError) / (t - 1);

      // Update old
      oldError = error;

      // Calculating the return value
      return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

    bool run(double setpoint, double precision = 1, double secondsAllowed = 2, int recursions = 5){
      
      // vars
      double current;

      // Loop of recursions
      for (int i = 0; i < recursions; i++){

        // Loop of ticks, converting seconds allowed into sets of 10 ms
        for (t = 0; t < (secondsAllowed * 100); t++){
          
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
          wait(10,msec); 
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