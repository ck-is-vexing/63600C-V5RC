#pragma once

#include <string>
#include "vex.h"

/**
 * @class Button
 * @brief Create a clickable button on VEX brain screen
 * 
 */
class Button {
  private:
    int xpos, ypos, x2pos, y2pos; // 2 corners of the button
    std::string textString;
    vex::color vexButtonColor, vexTextColor;
    vex::brain& br;
  public:

    /**
     * @brief Construct a new Button
     * 
     * @param x First corner x coordinate
     * @param y First corner y coordinate
     * @param x2 Second corner x coordinate
     * @param y2 Second corner y coordinate
     * @param text Text displayed inside the button
     * @param buttonColor VEX color for button
     * @param textColor VEX color for text
     * @param brain VEX brain object
     */
    Button(int x, int y, int x2, int y2, std::string text, vex::color buttonColor, vex::color textColor, vex::brain& brain);

    /**
     * @brief Render the button on brain screen
     * 
     * Run after every time the screen may be cleared
     * 
     */
    void render();

    /**
     * @brief Check if the button is currently being pressed
     * 
     * @return true if being pressed
     * @return false if not being pressed
     */
    bool isClicked();
};