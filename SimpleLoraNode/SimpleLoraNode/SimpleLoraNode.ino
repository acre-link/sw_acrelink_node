/*
  LoRa Simple Gateway/Node Exemple
  This code uses InvertIQ function to create a simple Gateway/Node logic.
  Gateway - Sends messages with enableInvertIQ()
          - Receives messages with disableInvertIQ()
  Node    - Sends messages with disableInvertIQ()
          - Receives messages with enableInvertIQ()
  With this arrangement a Gateway never receive messages from another Gateway
  and a Node never receive message from another Node.
  Only Gateway to Node and vice versa.
  This code receives messages and sends a message every second.
  InvertIQ function basically invert the LoRa I and Q signals.
  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on InvertIQ register 0x33.
  created 05 August 2018
  by Luiz H. Cassettari
*/

#include <SPI.h>              // include libraries
#include <LoRa.h>

const long frequency = 868E6;  // LoRa Frequency

const int csPin = 5;          // LoRa radio chip select
const int resetPin = 15;        // LoRa radio reset
const int irqPin = 4;          // change for your board; must be a hardware interrupt pin
const int spreadingFactor = 8;

void setup() {
  Serial.begin(9600);                   // initialize serial
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

  Serial.println("LoRa init succeeded.");


  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

void loop() {
  if (runEvery(10000)) { // repeat every 10000 millis

    String message = "HeLoRa World! ";
    message += "I'm a Node! ";
    message += millis();

    LoRa_sendMessage(message); // send a message

    Serial.println("Send Message!");

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
  LoRa_rxMode();
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
