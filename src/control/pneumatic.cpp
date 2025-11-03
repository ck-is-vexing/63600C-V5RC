#include "control/pneumatic.h"
#include "definition.h"

Pneumatic::Pneumatic(vex::triport::port& solenoidPort, bool reverse)
: solenoid(vex::digital_out(solenoidPort)), reversed(reverse), toggleCooldownTime(0) {}

bool Pneumatic::getValue() {
  const unsigned int value = solenoid.value();

  if (value == 1) {
    return true ^ reversed;
  } else {
    return false ^ reversed;
  }
}

void Pneumatic::toggle(bool override) {

  constexpr unsigned int COOLDOWN_MS = 350;
  
  if ((toggleCooldownTime + COOLDOWN_MS) < vex::timer::system()) {
    const unsigned int value = solenoid.value();
    toggleCooldownTime = vex::timer::system();

    if (value == 1) {
      solenoid.set(true ^ reversed);
    } else {
      solenoid.set(false ^ reversed);
    }

    if (override) {
      reversed = !reversed;
    }
  }
}

void Pneumatic::setTo(const bool value) {
  if (getValue() != value){
    solenoid.set(value ^ reversed);
  }
}