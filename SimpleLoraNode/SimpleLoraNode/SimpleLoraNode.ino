/*
  Simple LoRa Node
  Sends a message and goes to deep sleep after tx is done. 

  20210110
  by Maximilian Betz
 */

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <time.h>

const long frequency = 868E6;  // LoRa Frequency

const int misoPin = 19;
const int mosiPin = 23;
const int sckPin = 18;
const int csPin = 5;          // LoRa radio chip select
const int resetPin = 15;        // LoRa radio reset
const int irqPin = 4;          // change for your board; must be a hardware interrupt pin

const int spreadingFactor = 8;  //8 default
const int txPower = 17; //17 default 
const int sleepTimeS = 10;


void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  Serial.println("LoRa Node");

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.setSpreadingFactor(spreadingFactor);
  Serial.print("SpreadingFactor: ");
  Serial.println(spreadingFactor, DEC);

  LoRa.setTxPower(txPower);
  Serial.print("TxPower: ");
  Serial.println(txPower, DEC);
  
  Serial.println("LoRa init succeeded.");


  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

void loop() {
  String message = "HeLoRa World! ";
  message += "I'm a Node! ";
  message += millis();
  LoRa_sendMessage(message); // send a message
  Serial.print("Send Message!: ");
  Serial.println(message);

  while(true)
  {
    //Wait for tx Done and then go to sleep. 
  }
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  Serial.print("SendingMessage time_ms: ");
  Serial.println(millis(), DEC);
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
}

void onTxDone() {
  Serial.print("TxDone time_ms: ");
  Serial.println(millis(), DEC);

  Serial.print("Going to sleep for: ");
  Serial.print(sleepTimeS, DEC);
  Serial.println("s");
  LoRa.sleep();
  //LoRa_rxMode();  // Activate RX in case a receive window is desired.

  /*Go to sleep*/
  pinMode(misoPin, INPUT_PULLUP);
  pinMode(mosiPin, INPUT_PULLUP);
  pinMode(sckPin, INPUT_PULLUP);
  pinMode(csPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP); 
  esp_sleep_enable_timer_wakeup(1000000 * sleepTimeS); // Sleep 10 second
  esp_deep_sleep_start();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}
