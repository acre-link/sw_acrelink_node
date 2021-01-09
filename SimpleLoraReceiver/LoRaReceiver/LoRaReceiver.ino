#include <SPI.h>
#include <LoRa.h>

const long frequency = 868E6;  // LoRa Frequency

const int csPin = 5;          // LoRa radio chip select
const int resetPin = 15;        // LoRa radio reset
const int irqPin = 4;          // change for your board; must be a hardware interrupt pin
const int spreadingFactor = 8;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Receiver");

  LoRa.setPins(csPin, resetPin, irqPin);
  
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  LoRa.setSpreadingFactor(spreadingFactor);
  Serial.print("SpreadingFactor: ");
  Serial.println(spreadingFactor, DEC);

  Serial.println("LoRa init succeeded.");


  /*Dump Register to check default config*/
  //LoRa.dumpRegisters(Serial);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
