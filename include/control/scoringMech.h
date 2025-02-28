#ifndef SCORINGMECH_H
#define SCORINGMECH_H

#include "func/PID.h"
#include "vex.h"

// TODO: Add comments!
class wallStake {
  private:
    PID wsPID;
    double sp;

    vex::motor& mot;
    vex::rotation& rot;
  public:
    bool running;

    wallStake(vex::motor& motor, vex::rotation& rotationSensor);

    void setAngle(double angle);

    double angle(vex::rotationUnits unit = vex::rotationUnits::deg);

    void tick();

    void stop(vex::brakeType type = vex::brakeType::coast);

    void cancel();
};

#endif