#include <SPI.h>
#include <LoRa.h>

const long frequency = 868E6;  // LoRa Frequency

const int csPin = 5;          // LoRa radio chip select
const int resetPin = 15;        // LoRa radio reset
const int irqPin = 4;          // change for your board; must be a hardware interrupt pin
const int spreadingFactor = 10;

volatile bool txDoneFlag = false;
volatile bool rxDoneFlag = false;
volatile int rxPacketSize = 0;
volatile uint8_t rxMessageBuffer[30];
volatile int8_t rxRssi = 0;

/* Set flag if reception occured*/
void onReceiveDone(int packetSize) 
{
  for(int i = 0; i<packetSize; i++)
  {
    rxMessageBuffer[i] = LoRa.read();
  }
  rxRssi = LoRa.packetRssi();
  rxPacketSize = packetSize;
  rxDoneFlag = true;
}

/* Set flag once transmission done*/
void onTransmittDone() 
{
  txDoneFlag = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  
  //pinMode(irqPin, INPUT);   //TODO: test if this helps


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

  LoRa.onReceive(onReceiveDone);
  //LoRa.enableInvertIQ();
  
  //LoRa.onTxDone(onTransmittDone);

  LoRa.receive();
  

}

void loop() {


  if(rxDoneFlag == true)
  {
    Serial.print("Received something. Length: ");
    Serial.print(rxPacketSize, DEC);
    Serial.print(" Payload: "); 

    for(int i = 0; i < rxPacketSize; i++)
    {
      Serial.print(rxMessageBuffer[i], HEX);
      Serial.print(":");
    }
    Serial.print(" RSSI: ");
    Serial.println(rxRssi, DEC); 

    
    
    rxDoneFlag = false;

    /*Decode Message and Push here.*/
  }
  
  /*
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet. Length: ");
    Serial.print(packetSize, DEC);
    Serial.print(" :");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read(),HEX);
       Serial.print(":");
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  */
}
