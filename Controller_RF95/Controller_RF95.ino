#include "../RGB_LED.h"
#include "../Launcher.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <RH_RF95.h>

// RF95 Pin definitions
#define RFM95_CS    2
#define RFM95_RST   5
#define RFM95_INT   21

// Frequency
#define RF95_FREQ 915.0

// TFT SPI pins
#define TFT_SCLK   13
#define TFT_MOSI  12
#define TFT_CS    14
#define TFT_RST   32
#define TFT_DC    27
#define MAX_LINES 17

#define SERIAL_BAUD   115200

// Status LED PINs
#define LED_RED       33
#define LED_GREEN     26
#define LED_BLUE      25

// Button pins
#define SELECT_PIN    4
#define STATUS_PIN    15
#define ARM_PIN       34
#define LAUNCH_PIN    35

// Touch pins
int touchThresh = 0;
int lastTouch = 0;
int lastComm = 0;
#define DEBOUNCE_DELAY  500

// SPI pin definitions (if using non-default pins)
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define SPI_CS        2

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
rgbLED myLED(LED_RED, LED_GREEN, LED_BLUE);
int nodeState = STATE_BOOT;

// Calibrate touch for each pin
void calibrateTouchSensor(int myPin) {

  int baseline = 0;
  for (int i = 0; i < 100; i++) {
    baseline += touchRead(myPin);
    delay(10);
  }
  baseline /= 100;
  
  // Set threshold to 80% of baseline
  int threshold = baseline * 0.5;
  touchThresh = threshold;
  Serial.printf("Calibrated threshold for pin %d: %d\n", myPin, threshold);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(50);

  // Initialize the screen
  tft.init(135,240);
  tft.setRotation(1);
  tft.setTextSize(1);
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println("Waking up radio...");
  tft.println("Waking up radio...");

  // Configure our digital pins
  pinMode(ARM_PIN, INPUT);
  pinMode(LAUNCH_PIN, INPUT);
  pinMode(RFM95_RST, OUTPUT);
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
  tft.println("Finished waking up.");
  bool worked = rf95.init();
  delay(500);
  if (worked) {
    Serial.println("Radio initialized.");
    tft.println("Radio initialized.");
    myLED.setBlinkSpeed(BLINK_MED);
  } else {
    Serial.println("Radio failed to init.");
    tft.println("Radio failed to init.");
    myLED.setColor(ledState[STATE_DONE][0]);
    myLED.setBlinkSpeed(ledState[STATE_DONE][1]);
  }

  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(13, false);

  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", RF95_FREQ);
  Serial.println(buff);
  tft.println(buff);
    
  // Calibrate touch pins
  calibrateTouchSensor(STATUS_PIN);
}

// Received a link request
bool gotLink() {
  // Set to default state
  nodeState = STATE_BOOT;
  rf95.send((uint8_t*)"LINK", 4);
  rf95.waitPacketSent();
  Serial.println("Linked with remote");
  tft.println("Linked with remote");
  return true;
}

// Check for state
void getState() {
  rf95.send((uint8_t*)"STATE", 5);
  rf95.waitPacketSent();
}

// Send heartbeat
void sendHB() {
  rf95.send((uint8_t*)"HB", 2);
  rf95.waitPacketSent();
}

void gotState(String state) {
  // First parse out the last character into the status
  int digit = state[5] - '0';
  nodeState = digit;
  Serial.printf("Got state %s from remote\n", stateName[digit]);
  tft.printf("Got state %s from remote\n", stateName[digit]);
}

// Toggle the arming selector
void toggleArming() {
  if (nodeState == STATE_ARMED) {
    // Disarm
    Serial.println("Disarming node");
    tft.println("Disarming node");
    rf95.send((uint8_t*)"DISARM", 6);
    rf95.waitPacketSent();
  } else {
    // Try to arm
    Serial.println("Arming node");
    tft.println("Arming node");
    rf95.send((uint8_t*)"ARM", 3);
    rf95.waitPacketSent();
  }
}

void sendLaunch() {
  // Launch that puppy!
  rf95.send((uint8_t*)"LAUNCH", 6);
  rf95.waitPacketSent();
  tft.println("Sending launch to remote");
}

void loop() {

  String message = "";

  // Do we have a message to process?
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len)) {
      // Null terminate the received string
      buf[len] = 0;
      message = String((char*)buf);
    }    
  }

  /*
    There is a known universe of messages.  Specifically:

    "LINK"        The node is linked with us
    "HB"          The node sent a heartbeat
    "STATEx"      The node's current state (0-6 as defined in Launcher.h)
    "CONT"        Node has continuity
    "NOCONT"      Node has no continuity
    "ARMED"       Node is armed
    "NO_ARM"      Node is disarmed
    "LAUNCH"      Launched the rocket!
    "NOLAUNCH"    Attempted to launch but something (fatal) went wrong
  */

  // See what the message is
  if (message.startsWith("LINK")) {
    Serial.println("Linking with node");
    tft.println("Linking with node");
    // A node is attempting to link - if successful then get continuity
    if (gotLink()) {
      getState();
    }
  } else if (message.startsWith("HB")) {
    Serial.println("Heartbeat from node");
    sendHB();
  } else if (message.startsWith("STATE")) {
    gotState(message);
  } else if (message == "NOMSG") {
    // Nothing to do
  } else if (message != "") {
    // Got a message so get state again
    Serial.print("Received message from remote: ");
    tft.print("Received message from remote: ");
    Serial.println(message);    
    tft.println(message);
    getState();
  }

  /*
  Do basic housekeeping:
    * Set LED to the state of the selected remote
    * Reset the display
  */
  myLED.setColor(ledState[nodeState][0]);
  myLED.setBlinkSpeed(ledState[nodeState][1]);
  myLED.update();

  // Clear the screen if we're past the bottom
  int myY = tft.getCursorY();
  if (myY >= 135) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0,0);
  }

  // Debounce everybody (yes, lazy to not do this per input but I'll live with it for now)
  if (millis()-lastTouch < DEBOUNCE_DELAY) return;

  // See if the status pin is touched
  int touchValue = touchRead(STATUS_PIN);
  if (touchValue < touchThresh) {
    Serial.println("Getting status from remote");
    tft.println("Getting status for remote");
    lastTouch = millis();
    getState();
  }

  // See if the arm pin is pressed
  if (digitalRead(ARM_PIN) == HIGH) {
    lastTouch = millis();
    toggleArming();
  }

  // See if the launch pin is pressed
  if (digitalRead(LAUNCH_PIN) == HIGH) {    
    lastTouch = millis();
    sendLaunch();
  }

}
