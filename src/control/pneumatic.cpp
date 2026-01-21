#include "control/pneumatic.h"

Pneumatic::Pneumatic(vex::triport::port& solenoidPort, bool reverse, unsigned int cooldown)
: solenoid(vex::digital_out(solenoidPort)), reversed(reverse), toggleCooldownTime(0), clickCooldown(cooldown) {}

bool Pneumatic::getValue() {
  const unsigned int value = solenoid.value();

  if (value == 1) {
    return true ^ reversed;
  } else {
    return false ^ reversed;
  }
}

void Pneumatic::toggle(bool override) {

  if ((toggleCooldownTime + clickCooldown) < vex::timer::system()) {

    toggleCooldownTime = vex::timer::system();

    if (getValue() == true) {
      solenoid.set(false ^ reversed);
    } else {
      solenoid.set(true ^ reversed);
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

void Pneumatic::setCooldown(const unsigned int cooldown) {
  clickCooldown = cooldown;
}