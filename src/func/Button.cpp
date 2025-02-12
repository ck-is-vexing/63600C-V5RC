#include "vex.h" // Include VEX headers
#include <string> // Include string class

// General VEX brain button implementation
class Button {
  private:
    // Define variables
    int xpos, ypos, x2pos, y2pos; // Positions for the 2 corners of the button
    std::string textString; // The string to be printed on the button
    color vexButtonColor, vexTextColor; // The color of the button and the text
  public:
    // x, y, x2, and y2 are the coordinates for upper left and lower right corners of the button, respectively
    // text string goes roughly in the center of the button
    // buttonColor and textColor are vex colors
    Button(int x, int y, int x2, int y2, std::string text, color buttonColor, color textColor)
    : xpos(x), ypos(y), x2pos(x2), y2pos(y2), textString(text), vexButtonColor(buttonColor), vexTextColor(textColor) {}

    // Render the button onto the brain screen
    void render() {
      Brain.Screen.drawRectangle(xpos, ypos, (x2pos - xpos), (y2pos - ypos), vexButtonColor);
      Brain.Screen.setPenColor(vexTextColor);
      Brain.Screen.printAt((xpos + 10), (ypos + y2pos) / 2, false, textString.c_str());
    }

    // Returns true if there currently is a press on the button, otherwise returns false
    bool isClicked(){
      // Condition that checks if click occurred within the bounds of xpos, ypos, x2pos, and y2pos
      if (Brain.Screen.pressing() && Brain.Screen.xPosition() >= xpos && Brain.Screen.xPosition() <= x2pos && Brain.Screen.yPosition() >= ypos && Brain.Screen.yPosition() <= y2pos){
        return true;
      }
      return false;
    }
};