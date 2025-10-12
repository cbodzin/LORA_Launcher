#ifndef BUZZER_H
#define BUZZER_H

#define BUZZER_SHORT  250
#define BUZZER_MED    500
#define BUZZER_LONG   750
#define BUZZER_CHIRP  1500
#define CHIRP_SPEED   5

class buzzer {
private:
    int buzzPin;
    int buzzSpeed;        // milliseconds between blinks
    bool currentState;     // true = on, false = off
    bool currentCycle;
    bool chirping = false;
    unsigned long lastBuzzTime;
    unsigned long lastChirp;

public:
    // Constructor - initialize with pin numbers
    buzzer(int thePin) {
      buzzPin = thePin;
      buzzSpeed = BUZZER_SHORT;
      currentState = false;
      currentCycle = true;
      lastBuzzTime = 0;
      
      // Initialize pins as outputs
      pinMode(buzzPin, OUTPUT);
      
      // Start with LED off
      digitalWrite(buzzPin, LOW);
    }
    
    void on(int mySpeed = BUZZER_SHORT) {
      currentState = true;
      currentCycle = true;
      lastBuzzTime = millis();
      buzzSpeed = mySpeed;
      digitalWrite(buzzPin, HIGH);
    }
    
    void off() {
      currentState = false;
      digitalWrite(buzzPin, LOW);
    }
    
    void chirpOn() {
      chirping = true;
    }
    
    void chirpOff() {
      chirping = false;
    }
            
    // Set blink speed in milliseconds
    void setBuzzSpeed(int speedMs) {
      if (speedMs > 0) buzzSpeed = speedMs;
    }
    
    // Get current blink speed
    int getBuzzSpeed() {
      return buzzSpeed;
    }
    
    // Just do a set of beeps
    void doBeep(int numBeeps, int beepDuration) {
      for (int i = 0; i < numBeeps; i++) {
      	digitalWrite(buzzPin, HIGH);
      	delay(beepDuration);
      	digitalWrite(buzzPin, LOW);
      	// Skip pause on last iteration
      	if (i+1 < numBeeps) {
      	  delay(beepDuration);
      	}
      }
    }
    
    // This must be called in loop() to handle buzzing about
    void update() {
      if (currentState) {
        unsigned long currentTime = millis();
        if (currentTime - lastBuzzTime >= buzzSpeed) {
          currentCycle = !currentCycle;
          lastBuzzTime = currentTime;
        }   
        digitalWrite(buzzPin, currentCycle ? HIGH : LOW);
      } else {
        // Check to see if we should chirp; this is blocking
        if ((chirping) && (millis() - lastChirp >= BUZZER_CHIRP)) {
          lastChirp = millis();
          digitalWrite(buzzPin, HIGH);
      	  delay(CHIRP_SPEED);
      	  digitalWrite(buzzPin, LOW);
        }
      }
    }    
};

#endif