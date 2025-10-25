#include <SPI.h>
#include <RH_RF95.h>
#include "../RGB_LED.h"
#include "../Launcher.h"
#include "../Buzzer.h"

// RF95 Pin definitions
#define RFM95_CS    15
#define RFM95_RST   5
#define RFM95_INT   27

// Misc
#define SERIAL_BAUD   115200
#define LINKWAIT      500

// Status LED PINs
#define LED_RED     33
#define LED_GREEN   25
#define LED_BLUE    26
#define BUZZER_PIN  32
#define RELAY_PIN   13
#define MOSFET_GATE 17
#define PROBE_PIN   14
#define CONT_PIN    22

// SPI pin definitions (if using non-default pins) for ESP32-WROOM-32
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define SPI_CS        15

// Some global constants
int currentState = STATE_BOOT;
bool continuity = false;
bool armed = false;
int linkTry = 0;
int heartBeat = 0;
int hbFailed = 0;

// Global objects
RH_RF95 rf95(RFM95_CS, RFM95_INT);
rgbLED myLED(LED_RED, LED_GREEN, LED_BLUE);
buzzer myBuzz(BUZZER_PIN);

void setup() {
  // Set up serial output
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  Serial.println();
  Serial.println("Waking up radio...");

  // Configure our digital pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MOSFET_GATE, OUTPUT);
  pinMode(PROBE_PIN, INPUT_PULLUP);
  pinMode(CONT_PIN, INPUT_PULLUP);
  myLED.on(ledState[STATE_BOOT][0]);
  myLED.setBlinkSpeed(ledState[STATE_BOOT][1]);
  myLED.startBlink();

  // We wake up the RF95 
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  Serial.println("Finished waking up.");

  bool worked = rf95.init();
  delay(500);
  if (worked) {
    Serial.println("Radio initialized!");
    myLED.setBlinkSpeed(BLINK_MED);
  } else {
    Serial.println("Radio failed to init");
    currentState = STATE_DONE;
    myLED.setColor(ledState[STATE_DONE][0]);
    myLED.setBlinkSpeed(ledState[STATE_DONE][1]);
  }
  
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(RF95_TX_HIGH, false);

  char buff[50];
  sprintf(buff, "\nTransmitting at %f Mhz...", RF95_FREQ);
  Serial.println(buff);

  // Make sure relay and MOSFET are both low
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(MOSFET_GATE, LOW);

}

// Try to link with a gateway
void getLink() {
  Serial.println("Turning on");
  myBuzz.chirpOff();
  rf95.send((uint8_t*)"LINK", 4);
  rf95.waitPacketSent();
}

// Send a heartbeat
void getHB() {
  Serial.println("Sending hearbeat");
  rf95.send((uint8_t*)"HB", 2);
  rf95.waitPacketSent();
}

// We got a link message from a Gateway, so move to STATE_READY if we have continuity
void gotLink() {
  Serial.println("Linked with gateway");
  myBuzz.chirpOn();
  // Either go to READY or NOCONT depending on continuity
  continuity = getContinuity();
  if (continuity) {
    Serial.println("Setting ready");
    currentState = STATE_READY;    
  } else {
    currentState = STATE_NOCONT;
  }
  myLED.setColor(ledState[currentState][0]);
  myLED.setBlinkSpeed(ledState[currentState][1]);
  myLED.update();
  hbFailed = 0;
  // Remember when we're linked
  heartBeat = millis();
}

// Check continuity
bool getContinuity() {
  // Check sensor pin for signal
  bool contCheck = !digitalRead(PROBE_PIN);
  Serial.print("Continuity check: ");
  Serial.println(contCheck ? "Continuity" : "None");
  continuity = contCheck;
  if ((currentState == STATE_ARMED) && !contCheck) {
    currentState = STATE_NOCONT;
  }
  return contCheck;
}

// Send our current state
void sendState() {
  continuity = getContinuity();
  Serial.print("Sending state: ");
  Serial.println(currentState);
  char buff[6];
  sprintf(buff, "STATE%d", currentState);
  rf95.send((uint8_t*)buff, 6);
  rf95.waitPacketSent();
}

// Send our current arming status
void sendArmed() {
  Serial.print("Sending armed: ");
  Serial.println(armed);
  char buff[6];
  sprintf(buff, armed ? "ARMED " : "NO_ARM");
  rf95.send((uint8_t*)buff, 6);
  rf95.waitPacketSent();  
}

bool setArmed(bool newState) {
  if (!continuity && newState) {
    Serial.println("Cannot arm without continuity");
    return false;
  } 
  // Just skip if not linked
  if (currentState == STATE_BOOT) {
    return false;
  }
  armed = newState;  
  Serial.print("Setting armed to ");
  Serial.println(armed);
  if (armed && continuity) {
    // Turn on buzzer
    myBuzz.chirpOff();
    myBuzz.on(BUZZER_SHORT);
    // Turn on the MOSFET
    digitalWrite(MOSFET_GATE, HIGH);
    currentState = STATE_ARMED;
  } else {
    currentState = continuity ? STATE_READY : STATE_NOCONT;
    // Close Mosfet to disable relay
    digitalWrite(MOSFET_GATE, LOW);
    // Turn off buzzer
    myBuzz.off();
    myBuzz.chirpOn();
  }

  // Set the LED
  myLED.setColor(ledState[currentState][0]);
  myLED.setBlinkSpeed(ledState[currentState][1]);
  myLED.update();
  return true;
}

void doLaunch() {
  // Open the relay
  digitalWrite(RELAY_PIN, HIGH);
  // Make it long enough for me to pull the wire
  delay(1500);
  digitalWrite(RELAY_PIN, LOW);
  // Close Mosfet after short delay
  delay(250);
  digitalWrite(MOSFET_GATE, LOW);
  // Turn off buzzer
  myBuzz.off();
  // Assume it fired
  continuity = false;
  currentState = STATE_DONE;
}

bool setLaunch() {
  // First things first, this is only valid if we are armed and have continity
  if (!armed || !continuity) {
    Serial.println("Cannot launch unless armed and with continuity.");
    return(false);
  }
    
  // Launch!
  doLaunch();
  
  // Check continuity
  if (getContinuity()) {
    Serial.println("Fuck");
  } else {
    currentState = STATE_DONE;
  }
  // Send results to controller
  char buff[50];
  sprintf(buff, "LAUNCH Success");
  currentState = STATE_DONE;
  Serial.println(buff);
  rf95.send((uint8_t*)buff, strlen(buff));
  rf95.waitPacketSent();  
}

/*

  This is the main loop that is a state machine.  State progressions are purely linear!

  1) STATE_BOOT - power on, try to link with controller
  2) STATE_READY - linked and ready for commands
  3) STATE_ARMED - the controller has asked us to arm
  4) STATE_LAUNCH - we've been given the command to launch
  5) STATE_DONE - we no longer have continuity and cannot be used without a reset

*/

void loop() {

  String message = "";

  // Check for any received packets
  if (rf95.available() && currentState != STATE_DONE) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len)) {
      // Null terminate the received string
      buf[len] = 0;
      message = String((char*)buf);
    }

    /*
      There is a known universe of messages.  Specifically:

      "LINK"        The controller is linked with us
      "HB"          We got a heartbeat
      "STATE"       The controller wants to know our state
      "ARM"         Get ready to launch
      "DISARM"      Hold the launch and go back to STATE_READY
      "LAUNCH"      Launch the rocket!
    */

    // See what the message is
    if (message == "LINK") {
      hbFailed = 0;
      Serial.println("Gotlink");
      gotLink();
    } else if (message == "STATE") {
      // Send our current State
      hbFailed = 0;
      sendState();
    } else if (message == "ARM" ) {
      // Toggle arming
      bool result = setArmed(true);
    } else if (message == "DISARM") {
      bool result = setArmed(false);
    } else if (message == "LAUNCH") {
      if (currentState = STATE_ARMED) {
        currentState = STATE_LAUNCH;        
      }
      
    } else if (message == "HB") {
      Serial.println("Heartbeat received from controller");
      hbFailed = 0;
    } else if ((message == "") && (currentState > STATE_NOCONT)) {
      // We're not yet listening, just make sure we still have heartbeat if we haven't received anything
      Serial.println("Checking heartbeat...");
      if ((millis()-heartBeat) > HB_CHECK_TIME) {
        // First reset the heartbeat timer
        heartBeat = millis();
        if (currentState > STATE_NOCONT) hbFailed++;
        if (hbFailed <= MAX_HB_FAIL ) {
          // Try getting hearbeat
          getHB();
        } else {
          // We've failed enough that we need to go back to boot
          Serial.println("Too many failed heartbeats, resetting to STATE_BOOT");
          hbFailed = 0;
          linkTry = 0;
          setArmed(false);
          currentState = STATE_BOOT;
        }
      }
    } else if (message != "") {
      Serial.print("Received unsupported command: ");
      Serial.println(message);    
    }
  }
  
  // We're a state machine.  Big SWITCH statement for our state.
  switch (currentState) {
    case STATE_BOOT:
      // Everything starts here.  Try to link with the controller.
      if ((millis() - linkTry) > LINKWAIT) {
        linkTry = millis();
        Serial.println("Initial boot link try");
        getLink();
      }
      break;
    case STATE_READY:
    case STATE_NOCONT:
    case STATE_ARMED:
      if ((millis()-heartBeat) > HB_CHECK_TIME) {
        Serial.println("Checking heartbeat...");
        // First reset the heartbeat timer
        heartBeat = millis();
        if (currentState > STATE_NOCONT) hbFailed++;
        if (hbFailed <= MAX_HB_FAIL ) {
          // Try getting hearbeat
          Serial.printf("Missed %d heartbeats, trying again.\n", hbFailed);
          getHB();
        } else {
          // We've failed enough that we need to go back to boot
          Serial.println("Too many failed heartbeats, resetting to STATE_BOOT");
          hbFailed = 0;
          linkTry = 0;
          setArmed(false);
          currentState = STATE_BOOT;
        }
      }
      break;
    case STATE_LAUNCH:
      setLaunch();
      break;
    case STATE_DONE:
      myLED.setColor(ledState[currentState][0]);
      myLED.setBlinkSpeed(ledState[currentState][1]);
      myLED.update();
      myBuzz.off();
      myBuzz.chirpOff();
      Serial.println("Done!");
      sendState();
      delay(2000);
      break;
    default:
      // We should never be here.  Something is horribly wrong.
      Serial.println("In undetermined state!");
      myLED.setColor(STATE_DONE);
      myLED.setBlinkSpeed(BLINK_FAST);
      break;
  }
  /*
  Do basic housekeeping:
  * Blink LED
  * Do buzzer
  */
  myLED.setColor(ledState[currentState][0]);
  myLED.setBlinkSpeed(ledState[currentState][1]);
  myLED.update();

  // Is the continuity pin pressed?
  if (digitalRead(CONT_PIN) == LOW) {
    bool contCheck = getContinuity();
    while (digitalRead(CONT_PIN) == LOW) {
      if (contCheck) {
        myBuzz.doBeep(1, 500);
      } else {
        myBuzz.doBeep(3, 50);
      }
    }
  }
  myBuzz.update();
  
}