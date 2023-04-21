#ifndef LED_INDICATORS_H
#define LED_INDICATORS_H

#include <Arduino.h>

#define COLOR_BLACK 0x000000
#define COLOR_WHITE 0xFFFFFF
#define COLOR_RED 0xFF0000
#define COLOR_GREEN 0x00FF00
#define COLOR_BLUE 0x0000FF
#define COLOR_CYAN 0x00FFFF
#define COLOR_MAGENTA 0xFF00FF
#define COLOR_YELLOW 0xFFFF00

class LED {
  public:
    LED(int pin); // Constructor
    void blink(uint32_t color, uint16_t duration); // Method to blink the LED with a specific color and duration
    void on(uint32_t color); // Method to turn the LED on with a specific color
    void off(); // Method to turn the LED off
  private:
    int _pin; // Private member variable to store the LED pin number
};


#endif
