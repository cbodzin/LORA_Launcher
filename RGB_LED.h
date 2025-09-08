#ifndef RGB_LED_H
#define RGB_LED_H

// On/off combinations for status LED
bool rgbColors[8][3] = {
  {HIGH, LOW,  LOW},   // Red
  {LOW,  HIGH, LOW},   // Green
  {LOW,  LOW,  HIGH},  // Blue
  {LOW,  HIGH, HIGH},  // Cyan (Green + Blue)
  {HIGH, HIGH, LOW},   // Yellow (Red + Green)
  {HIGH, LOW,  HIGH},  // Magenta (Red + Blue)
  {HIGH, HIGH, HIGH},  // White (Red + Green + Blue)
  {LOW,  LOW,  LOW}    // Off
};

bool getColor(int color, int led) {
  return rgbColors[color][led];
}

// Easy way to get colors
#define RGB_RED     0
#define RGB_GREEN   1
#define RGB_BLUE    2
#define RGB_CYAN    3
#define RGB_YELLOW  4
#define RGB_MAGENTA 5
#define RGB_WHITE   6
#define RGB_OFF     7

// Blink speeds 
#define BLINK_SLOW  1000
#define BLINK_MED   500
#define BLINK_FAST  250
#define BLINK_NONE  0

class rgbLED {
private:
    int redPin;
    int greenPin;
    int bluePin;
    int blinkSpeed;        // milliseconds between blinks
    bool isBlinking;
    bool currentState;     // true = on, false = off
    unsigned long lastBlinkTime;
    bool redState, greenState, blueState;  // current color state

public:
    // Constructor - initialize with pin numbers
    rgbLED(int rPin, int gPin, int bPin) {
      redPin = rPin;
      greenPin = gPin;
      bluePin = bPin;
      blinkSpeed = BLINK_MED;      
      isBlinking = false;
      currentState = false;
      lastBlinkTime = 0;
      redState = false;
      greenState = false;
      blueState = false;
      
      // Initialize pins as outputs
      pinMode(redPin, OUTPUT);
      pinMode(greenPin, OUTPUT);
      pinMode(bluePin, OUTPUT);
      
      // Start with LED off
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
    }
    
    // Turn LED on with specified color
    // void on(bool red = true, bool green = true, bool blue = true) {
    void on(int myColor) {
      bool red = getColor(myColor, RGB_RED);
      bool green = getColor(myColor, RGB_GREEN);
      bool blue = getColor(myColor, RGB_BLUE);
      isBlinking = false;
      redState = red;
      greenState = green;
      blueState = blue;
      currentState = true;
      updateLed();
    }
    
    // Turn LED off
    void off() {
      isBlinking = false;
      currentState = false;
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
    }
    
    // Set specific color without changing on/off state
    // void setColor(bool red, bool green, bool blue) {
    void setColor(int myColor) {
      bool red = getColor(myColor, RGB_RED);
      bool green = getColor(myColor, RGB_GREEN);
      bool blue = getColor(myColor, RGB_BLUE);
      redState = red;
      greenState = green;
      blueState = blue;
      if (currentState && !isBlinking) {
          updateLed();
      }
    }
    
    // Start blinking with current color
    void startBlink() {
      isBlinking = true;
      lastBlinkTime = millis();
    }
        
    // Stop blinking and turn off
    void stopBlink() {
      isBlinking = false;
      off();
    }
    
    // Set blink speed in milliseconds
    void setBlinkSpeed(int speedMs) {
      blinkSpeed = speedMs;
    }
    
    // Get current blink speed
    int getBlinkSpeed() {
      return blinkSpeed;
    }
    
    // This must be called in loop() to handle blinking
    void update() {
      if (isBlinking) {
        unsigned long currentTime = millis();
        if (currentTime - lastBlinkTime >= blinkSpeed) {
          currentState = !currentState;
          lastBlinkTime = currentTime;
            
          if (currentState) {
              updateLed();
          } else {
              digitalWrite(redPin, LOW);
              digitalWrite(greenPin, LOW);
              digitalWrite(bluePin, LOW);
          }
        }
      }
    }
    
    // Check if LED is currently blinking
    bool getBlinkStatus() {
      return isBlinking;
    }
    
    // Check if LED is currently on (useful for debugging)
    bool getStatus() {
      return currentState;
    }

private:
    // Internal function to update LED with current color state
    void updateLed() {
        digitalWrite(redPin, redState ? HIGH : LOW);
        digitalWrite(greenPin, greenState ? HIGH : LOW);
        digitalWrite(bluePin, blueState ? HIGH : LOW);
    }
};

#endif

// Example usage:
/*
#include "RgbLed.h"

// Create RGB LED object with pins 2, 4, 5
RgbLed myLed(2, 4, 5);

void setup() {
    // LED is ready to use after construction
    
    // Turn on white (all colors)
    myLed.on();
    delay(1000);
    
    // Turn on red only
    myLed.on(true, false, false);
    delay(1000);
    
    // Turn on purple (red + blue)
    myLed.on(true, false, true);
    delay(1000);
    
    // Start blinking green at default speed (500ms)
    myLed.startBlink(false, true, false);
    
    // Change blink speed to 200ms
    myLed.setBlinkSpeed(200);
}

void loop() {
    // Must call update() in loop for blinking to work
    myLed.update();
    
    // Other code can go here...
}
*/