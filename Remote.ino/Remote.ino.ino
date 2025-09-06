#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69

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
#define LED_PIN       26
#define IRQ_PIN       27
#define LIFE_LED      25
#define TOUCH_PIN     32
#define RF69_IRQ_PIN  27
#define RF69_SPI_CS   15


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
bool lightState = false;

const int touchThreshold = 40;

void IRAM_ATTR rfm69Interrupt() {
  // Keep ISR as short as possible!
  // Just set a flag - do processing in main loop
  Serial.println("Got packet");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("Waking up radio...");
  pinMode(RST_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LIFE_LED, OUTPUT);
  pinMode(IRQ_PIN,INPUT);
  attachInterrupt(IRQ_PIN, rfm69Interrupt, RISING);
  digitalWrite(RST_PIN,HIGH);
  delay(500);
  digitalWrite(RST_PIN,LOW);
  delay(500);
  Serial.println("Finished waking up.");
  bool worked = radio.initialize(FREQUENCY,NODEID,NETWORKID);
  delay(500);
  if (worked) {
    Serial.println("Radio initialized.");
  } else {
    Serial.println("Radio failed to init.");
  }

  radio.setIrq(IRQ_PIN);
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

byte ackCount=0;
uint32_t packetCount = 0;

unsigned long previousTime = 0;
const unsigned long interval = 500;  // 500ms
int toggle = 0;
  
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime > interval) {
    toggle = 1 - toggle;
    digitalWrite(LIFE_LED, toggle);
    previousTime = currentTime;
  }

  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == 'r') //d=dump all register values
      radio.readAllRegs();
    if (input == 'E') //E=enable encryption
      radio.encrypt(ENCRYPTKEY);
    if (input == 'e') //e=disable encryption
      radio.encrypt(null);
    if (input == 'p')
    {
      spy = !spy;
      radio.spyMode(spy);
      Serial.print("SpyMode mode ");Serial.println(spy ? "on" : "off");
    }
    
    if (input == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
      Serial.print( "Radio Temp is ");
      Serial.print(temperature);
      Serial.print("C, ");
      Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
      Serial.println('F');
    }

    if (input == 'z') {
      Serial.print("Pinging...");
      radio.sendWithRetry(2, "TEST", 4);
      Serial.println("done!");
    }

    if (input == 'l') {
      Serial.println("Lightstate toggle");
      lightState = !lightState;
      digitalWrite(LED_PIN, lightState);
    }

  }

  int touchValue = touchRead(TOUCH_PIN);
  if (touchValue < touchThreshold) {
    Serial.println("Touch detected");
    lightState = !lightState;
    digitalWrite(LED_PIN, lightState);
    delay(500);
  }

  if (radio.receiveDone())
  {
    String message = "";
    Serial.print("#[");
    Serial.print(++packetCount);
    Serial.print(']');
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    if (spy) Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print((char)radio.DATA[i]);
      message = message + (char)radio.DATA[i];
    }
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
    Serial.println();

    if (message == "ON") {
      Serial.println("Turning on LED");
      lightState = true;
    } else if (message == "OFF") {
      Serial.println("Turning off LED");
      lightState = false;
    } else {
      Serial.print("Got weird message: ");
      Serial.println(message);
    }
    digitalWrite(LED_PIN, lightState);
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
    }
    Serial.println();
  }
}
