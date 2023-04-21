#include "led_indicators.h"

LED::LED(int pin) {
  _pin = pin;
  pinMode(_pin, OUTPUT);
}

void LED::blink(uint32_t color, uint16_t duration) {
  on(color); // Turn the LED on with the specified color
  delay(duration); // Wait for the specified duration
  off(); // Turn the LED off
  delay(duration); // Wait for the specified duration again
}

void LED::on(uint32_t color) {
  digitalWrite(_pin, HIGH); // Turn the LED on
  analogWrite(LED_RED, (color >> 16) & 0xFF); // Set the red channel using bit shifting and masking
  analogWrite(LED_GREEN, (color >> 8) & 0xFF); // Set the green channel using bit shifting and masking
  analogWrite(LED_BLUE, color & 0xFF); // Set the blue channel using masking
}

void LED::off() {
  digitalWrite(_pin, LOW); // Turn the LED off
  analogWrite(LED_RED, 0); // Set the red channel to 0
  analogWrite(LED_GREEN, 0); // Set the green channel to 0
  analogWrite(LED_BLUE, 0); // Set the blue channel to 0
}