#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include "../RGB_LED.h"
#include "../Launcher.h"

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        1    //should always be 1 for a Gateway
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************
#define SERIAL_BAUD   115200
#define RST_PIN       5
#define IRQ_PIN       27
#define RF69_IRQ_PIN  27
#define RF69_SPI_CS   15

// Status LED PINs
#define LED_RED       33
#define LED_GREEN     26
#define LED_BLUE      25

// Button pins
#define SELECT_PIN    32
#define STATUS_PIN    14
#define ARM_PIN       12
#define LAUNCH_PIN    13

// SPI pin definitions (if using non-default pins)
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define SPI_CS        15

#ifdef ENABLE_ATC
  RFM69_ATC radio(RF69_SPI_CS, RF69_IRQ_PIN, true, null);
#else
  RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, true, null);
#endif

bool spy = true; //set to 'true' to sniff all packets on the same network
rgbLED myLED(LED_RED, LED_GREEN, LED_BLUE);
const int touchThreshold = 40;
bool havePacket = false;

// Array to hold node states; NODEID will be the index
int nodeState[5];
int currentNode = 2;    // Allow this to be selected later

void IRAM_ATTR rfm69Interrupt() {
  // Keep ISR as short as possible!
  // Just set a flag - do processing in main loop
  havePacket = true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Waking up radio...");

  // Configure our digital pins
  pinMode(RST_PIN, OUTPUT);
  myLED.on(ledState[STATE_BOOT][0]);
  myLED.setBlinkSpeed(ledState[STATE_BOOT][1]);
  myLED.startBlink();

  pinMode(RST_PIN, OUTPUT);
  pinMode(IRQ_PIN,INPUT);
  //attachInterrupt(IRQ_PIN, rfm69Interrupt, RISING);

  // We wake up the RFM69 by setting to high and then low.  
  digitalWrite(RST_PIN,HIGH);
  delay(500); // this is probably 10x as long as it needs to be, but no harm in waiting
  digitalWrite(RST_PIN,LOW);
  delay(500);
  Serial.println("Finished waking up.");
  bool worked = radio.initialize(FREQUENCY,NODEID,NETWORKID);
  delay(500);
  if (worked) {
    Serial.println("Radio initialized.");
    myLED.setBlinkSpeed(BLINK_MED);
  } else {
    Serial.println("Radio failed to init.");
    myLED.setColor(ledState[STATE_DONE][0]);
    myLED.setBlinkSpeed(ledState[STATE_DONE][1]);
  }

  //radio.setIrq(IRQ_PIN);

#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.spyMode(spy);
  //radio.setFrequency(916000000); //set frequency to some custom frequency
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
    
#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif

}

// Received a link request
bool gotLink(int nodeID) {
  // Set to default state
  nodeState[nodeID] = STATE_BOOT;
  radio.sendWithRetry(nodeID, "LINK", 4);
  Serial.printf("Linked with node %d\n", nodeID);
  return true;
}

// Check for state
void getState(int nodeID) {
  Serial.println("Sending state");
  radio.sendWithRetry(nodeID, "STATE", 5);
}

void gotState(int nodeID, String state) {
  // First parse out the last character into the status
  Serial.println("Getting Digit");
  int digit = state[5] - '0';
  nodeState[nodeID] = digit;
}

// Toggle the arming selector
void toggleArming(int nodeID) {
  if (nodeState[nodeID] == STATE_ARMED) {
    // Disarm
    Serial.printf("Disarming node %d\n", nodeID);
    radio.sendWithRetry(nodeID, "DISARM", 6);
  } else {
    // Try to arm
    Serial.printf("Arming node %d\n", nodeID);
    radio.sendWithRetry(nodeID, "ARM", 3);
  }
}

void sendLaunch(int nodeID) {
  // Launch that puppy!
  radio.sendWithRetry(nodeID, "LAUNCH", 6);
}

void loop() {

  String message = "";
  int myNode = -1;

  // Do we have a message to process?
  if (radio.receiveDone()) {
    Serial.println("Got Data");
    myNode = radio.SENDERID;
    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print((char)radio.DATA[i]);
      message = message + (char)radio.DATA[i];
    }
    Serial.println();
    Serial.println("Done.");
    Serial.print("Message is: ");
    Serial.println(message);

    if (radio.ACKRequested()) {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.println(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
    }
  }

  /*
    There is a known universe of messages.  Specifically:

    "LINK"        The node is linked with us
    "STATEx"      The node's current state (0-6 as defined in Launcher.h)
    "CONT"        Node has continuity
    "NOCONT"      Node has no continuity
    "ARMED"       Node is armed
    "NO_ARM"      Node is disarmed
    "LAUNCH"      Launched the rocket!
    "NOLAUNCH"    Attempted to launch but something (fatal) went wrong
  */

  // Make sure we're willing to talk with this node ID
  if (myNode > 0) {
    if ((myNode < MIN_NODE) || (myNode > MAX_NODE )) {
      Serial.printf("Ignoring message from out of bounds node %d\n", myNode);
      message = "NOMSG";
    }
  }

  // See what the message is
  if (message.startsWith("LINK")) {
    Serial.printf("Linking with node %d\n", myNode);
    // A node is attempting to link - if successful then get continuity
    if (gotLink(myNode)) {
      Serial.println("Getting state");
      getState(myNode);
    }
  } else if (message.startsWith("STATE")) {
    Serial.println("Recieved state");
    gotState(myNode, message);
  } else if (message == "NOMSG") {
    // Nothing to do
  } else if (message != "") {
    // Got a message so get state again
    Serial.printf("Received message from node %d: ", myNode);
    Serial.println(message);    
    getState(myNode);
  }

  /*
  Do basic housekeeping:
    * Set LED to the state of the selected remote
  */
  myLED.setColor(ledState[nodeState[currentNode]][0]);
  myLED.setBlinkSpeed(ledState[nodeState[currentNode]][1]);
  myLED.update();

  // See if the selector pin is touched
  int touchValue = touchRead(SELECT_PIN);
  if (touchValue < touchThreshold) {
    Serial.println("Selection detected");
    currentNode++;
    // Time to loop?
    if (currentNode > MAX_NODE) {
      currentNode = MIN_NODE;
    }
    getState(currentNode);
  }

  // See if the status pin is touched
  touchValue = touchRead(STATUS_PIN);
  if (touchValue < touchThreshold) {
    Serial.println("Status detected");
    getState(currentNode);
  }

  // See if the arm pin is touched
  touchValue = touchRead(ARM_PIN);
  if (touchValue < touchThreshold) {
    Serial.println("Arm detected");
    toggleArming(currentNode);
  }

  // See if the launch pin is touched
  touchValue = touchRead(LAUNCH_PIN);
  if (touchValue < touchThreshold) {
    Serial.println("Launch detected");
    sendLaunch(currentNode);
  }

}
