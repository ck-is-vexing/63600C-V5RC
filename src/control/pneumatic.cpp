#include "control/pneumatic.h"
#include "definition.h"

Pneumatic::Pneumatic(vex::triport::port& solenoidPort, const bool reverse)
: solenoid(vex::digital_out(solenoidPort)), reversed(reverse), toggleCooldownTime(0) {}

bool Pneumatic::getValue() {
  const unsigned int value = solenoid.value();

  if (value == 1) {
    return true ^ reversed;
  } else {
    return false ^ reversed;
  }
}

void Pneumatic::toggle() {
  printl(toggleCooldownTime);
  printl(vex::timer::system());

  constexpr unsigned int COOLDOWN_MS = 100;

  if ((toggleCooldownTime + COOLDOWN_MS) < vex::timer::system()) {
    const unsigned int value = solenoid.value();
    toggleCooldownTime = vex::timer::system();

    if (value == 1) {
      solenoid.set(true ^ reversed);
    } else {
      solenoid.set(false ^ reversed);
    }
  }
}

void Pneumatic::setTo(const bool value) {
  if (getValue() != value){
    solenoid.set(value ^ reversed);
  }
}