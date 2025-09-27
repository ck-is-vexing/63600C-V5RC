#include "func/button.h"

Button::Button(int x, int y, int x2, int y2, std::string text, vex::color buttonColor, vex::color textColor, vex::brain& brain)
: xpos(x), ypos(y), x2pos(x2), y2pos(y2), textString(text), vexButtonColor(buttonColor), vexTextColor(textColor), br(brain) {}

void Button::render() {
  br.Screen.drawRectangle(xpos, ypos, (x2pos - xpos), (y2pos - ypos), vexButtonColor);
  br.Screen.setPenColor(vexTextColor);
  br.Screen.printAt((xpos + 10), (ypos + y2pos) / 2, false, textString.c_str());
}

bool Button::isClicked(){

  if (br.Screen.pressing() && 
      br.Screen.xPosition() >= xpos && 
      br.Screen.xPosition() <= x2pos && 
      br.Screen.yPosition() >= ypos && 
      br.Screen.yPosition() <= y2pos){
    return true;
  }

  return false;
}