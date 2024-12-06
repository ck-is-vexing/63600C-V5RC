#include "vex.h" // Include VEX headers
#include <string>

class Button {
  private:
    int xpos, ypos, x2pos, y2pos;
    std::string textString;
    color vexButtonColor, vexTextColor;
  public:
    Button(int x, int y, int x2, int y2, std::string text, color buttonColor, color textColor){
      xpos = x;
      ypos = y;
      x2pos = x2;
      y2pos = y2;
      textString = text;
      vexButtonColor = buttonColor;
      vexTextColor = textColor;
    }

    void render() {
      Brain.Screen.drawRectangle(xpos, ypos, (x2pos - xpos), (y2pos - ypos), vexButtonColor);
      Brain.Screen.setPenColor(vexTextColor);
      Brain.Screen.printAt((xpos + 10), (ypos + y2pos) / 2, textString.c_str());
    }

    bool isClicked(){

      // Condition that checks if click occured within the bounds of xpos, ypos, x2pos, and y2pos
      if (Brain.Screen.pressing() && Brain.Screen.xPosition() >= xpos && Brain.Screen.xPosition() <= x2pos && Brain.Screen.yPosition() >= ypos && Brain.Screen.yPosition() <= y2pos){
        return true;
      }
      return false;
    }
};