#ifndef BUTTON_H
#define BUTTON_H

#include <string>
#include "vex.h"

// TODO: add comments
// General VEX brain button implementation
class Button {
  private:
    int xpos, ypos, x2pos, y2pos; // Positions for the 2 corners of the button
    std::string textString; // The string to be printed on the button
    vex::color vexButtonColor, vexTextColor; // The color of the button and the text
    vex::brain& br;
  public:
    // x, y, x2, and y2 are the coordinates for upper left and lower right corners of the button, respectively
    // text string goes roughly in the center of the button
    // buttonColor and textColor are vex colors
    Button(int x, int y, int x2, int y2, std::string text, vex::color buttonColor, vex::color textColor, vex::brain& brain);

    void render();

    // Returns true if there currently is a press on the button, otherwise returns false
    bool isClicked();
};

#endif