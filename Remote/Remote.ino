#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include "../RGB_LED.h"
#include "../Launcher.h"
#include "../Buzzer.h"

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
// Address IDs are 10bit, meaning usable ID range is 1..1023
// Address 0 is special (broadcast), messages to address 0 are received by all *listening* nodes (ie. active RX mode)
// Gateway ID should be kept at ID=1 for simplicity, although this is not a hard constraint
//*********************************************************************************************
#define NODEID        2    // keep UNIQUE for each node on same network
#define NETWORKID     100  // keep IDENTICAL on all nodes that talk to each other
#define GATEWAYID     1    // "central" node

//*********************************************************************************************
// Frequency should be set to match the radio module hardware tuned frequency,
// otherwise if say a "433mhz" module is set to work at 915, it will work but very badly.
// Moteinos and RF modules from LowPowerLab are marked with a colored dot to help identify their tuned frequency band,
// see this link for details: https://lowpowerlab.com/guide/moteino/transceivers/
// The below examples are predefined "center" frequencies for the radio's tuned "ISM frequency band".
// You can always set the frequency anywhere in the "frequency band", ex. the 915mhz ISM band is 902..928mhz.
//*********************************************************************************************
//#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define FREQUENCY_EXACT 916000000 // you may define an exact frequency/channel in Hz
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
//*********************************************************************************************
#define SERIAL_BAUD   115200
#define RST_PIN       5
#define IRQ_PIN       27
#define RF69_IRQ_PIN  27
#define RF69_SPI_CS   15

// Status LED PINs
#define LED_RED     33
#define LED_GREEN   26
#define LED_BLUE    25
#define BUZZER_PIN  32
#define RELAY_PIN   13

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

#define LINKWAIT  500

#ifdef ENABLE_ATC
  RFM69_ATC radio(RF69_SPI_CS, RF69_IRQ_PIN, true, null);
#else
  RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, true, null);
#endif

bool spy = false;
rgbLED myLED(LED_RED, LED_GREEN, LED_BLUE);
buzzer myBuzz(BUZZER_PIN);

void setup() {
  // Set up serial output
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(200);
  Serial.println();
  Serial.println("Waking up radio...");

  // Configure our digital pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  myLED.on(ledState[STATE_BOOT][0]);
  myLED.setBlinkSpeed(ledState[STATE_BOOT][1]);
  myLED.startBlink();

  // We wake up the RFM69 by setting to high and then low.  
  digitalWrite(RST_PIN,HIGH);
  delay(500); // this is probably 10x as long as it needs to be, but no harm in waiting
  digitalWrite(RST_PIN,LOW);
  delay(500);
  Serial.println("Finished waking up.");

  bool worked = radio.initialize(FREQUENCY,NODEID,NETWORKID);
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

#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.spyMode(spy);

// #ifdef FREQUENCY_EXACT
//   radio.setFrequency(FREQUENCY_EXACT); //set frequency to some custom frequency
// #endif
  
//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

}

// Try to link with a gateway
void getLink() {
  Serial.println("Turning on");
  myBuzz.chirpOff();
  radio.sendWithRetry(GATEWAYID, "LINK", 4);
}

void getHB() {
  Serial.println("Sending hearbeat");
  radio.sendWithRetry(GATEWAYID, "HB", 2);
}


// We got a link message from a Gateway, so move to STATE_READY if we have continuity
void gotLink(int gatewayID) {
  Serial.printf("Linked with gateway %d\n", gatewayID);
  myBuzz.chirpOn();
  // Did we relink as a failed heartbeat?
  if (hbFailed == 0) {   // No, not a failed heartbeat   
    // Either go to READY or NOCONT depending on continuity
    if (continuity) {
      Serial.println("Setting ready");
      currentState = STATE_READY;    
    } else {
      currentState = STATE_NOCONT;
    }
    myLED.setColor(ledState[currentState][0]);
    myLED.setBlinkSpeed(ledState[currentState][1]);
    myLED.update();
  } else {
    // Reset hbFailed
    hbFailed = 0;
  }
  // Remember when we're linked
  heartBeat = millis();
}

// Check continuity
bool getContinuity() {
  // TODO - check for it
  //bool result = (currentState == STATE_LAUNCH ? false : true);
  return true;
}

// Send our current state
void sendState() {
  Serial.print("Sending state: ");
  Serial.println(currentState);
  char buff[6];
  sprintf(buff, "STATE%d", currentState);
  radio.sendWithRetry(GATEWAYID, buff, 6);
}

// Send our current arming status
void sendArmed() {
  Serial.print("Sending armed: ");
  Serial.println(armed);
  char buff[6];
  sprintf(buff, armed ? "ARMED " : "NO_ARM");
  radio.sendWithRetry(GATEWAYID, buff, 6);
}

bool setArmed(bool newState) {
  if (!continuity && newState) {
    Serial.println("Cannot arm without continuity");
    return false;
  } 
  Serial.print("Setting armed to ");
  Serial.println(armed);
  if (armed) {
    myBuzz.chirpOff();
    myBuzz.on(BUZZER_SHORT);
  } else {
    myBuzz.off();
    myBuzz.chirpOn();
  }
  armed = newState;
  if (continuity && armed) {
    currentState = STATE_ARMED;
  } else if (!armed) {
    currentState = STATE_READY;
  }
  myLED.setColor(ledState[currentState][0]);
  myLED.setBlinkSpeed(ledState[currentState][1]);
  myLED.update();
  return true;
}

bool doLaunch() {
  // TODO - open relay and launxh, then delay some amount of time
  currentState = STATE_LAUNCH;
  // Open the relay
  digitalWrite(RELAY_PIN, HIGH);
  delay(500);
  digitalWrite(RELAY_PIN, LOW);
  myBuzz.off();
  return(true);
}

bool setLaunch() {
  // First things first, this is only valid if we are armed and have continity
  if (!armed || !continuity) {
    Serial.println("Cannot launch unless armed and with continuity.");
    return(false);
  }
  
  currentState = STATE_LAUNCH;
  
  // Launch!
  bool success = doLaunch();

  // We should have no conituity now that we fired
  bool igniterFired = getContinuity();
  
  // Send results to controller
  char buff[50];
  if (success && igniterFired) {
    sprintf(buff, "LAUNCH Success on remote %d", NODEID);
    currentState = STATE_DONE;
  } else if (success) {
    // Hmm, we opened the relay but there is still continuity
    sprintf(buff, "NOLAUNCH Relay fired but still continuity on remote %d", NODEID);
    currentState = STATE_ARMED;
  } else {
    // Couldn't fire
    sprintf(buff, "NOLAUCNH Failed relay on remote %d", NODEID);
    currentState = STATE_ARMED;
  }
  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
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
  int myNode = -1;

  // Check for any received packets
  if (radio.receiveDone()) {

    myNode = radio.SENDERID;
    // We got one - grab the data
    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print((char)radio.DATA[i]);
      message = message + (char)radio.DATA[i];
    }

    // Send an ACK if requested
    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println(" - ACK sent");
    }
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

  // Check continuity
  continuity = getContinuity();

  // See what the message is
  if (message == "LINK") {
    Serial.println("Gotlink");
    gotLink(myNode);
  } else if (message == "STATE") {
    // Send our current State
    sendState();
  } else if (message == "ARM" || message == "DISARM") {
    // Toggle arming
    armed = !armed;
    bool result = setArmed(armed);
  // in the future we may want explicit DISARM command
  // } else if (message == "DISARM") {
  //   bool result = setArmed(false);
  } else if (message == "LAUNCH") {
    setLaunch();
  } else if (message == "HB") {
    Serial.printf("Heartbeat received from %d\n",myNode);
    hbFailed = 0;
  } else if ((message == "") && (currentState != STATE_BOOT)) {
    // We're not yet listening, just make sure we still have heartbeat if we haven't received anything
    if ((millis()-heartBeat) > HB_CHECK_TIME) {
      // First reset the heartbeat timer
      heartBeat = millis();
      if (currentState != STATE_BOOT) hbFailed++;
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
    case STATE_ARMED:
      // Get continuity
      continuity = getContinuity();
      break;
    case STATE_LAUNCH:
      setLaunch();
      break;
    case STATE_DONE:
      myLED.setColor(ledState[currentState][0]);
      myLED.setBlinkSpeed(BLINK_NONE);
      myLED.update();
      myBuzz.off();
      myBuzz.chirpOn();
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

  myBuzz.update();
  
}