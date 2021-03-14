#include <SPI.h>
#include <LoRa.h>

const long frequency = 868E6;  // LoRa Frequency

const int csPin = 5;          // LoRa radio chip select
const int resetPin = 15;        // LoRa radio reset
const int irqPin = 4;          // change for your board; must be a hardware interrupt pin
const int spreadingFactor = 10;
const int txPower = 14; //14 is the legal limit on 868.0 - 868.7

/* Global objects */
bool rxDoneFlag = false;
int packageSize = 0;

/* Set LoRa to rx mode.  onReceive will be triggerd after a reception.*/
void LoRa_rxMode()
{
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}


void setup() {
  Serial.begin(115200);
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

  LoRa.setTxPower(txPower);
  Serial.print("TxPower: ");
  Serial.println(txPower, DEC);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  
  LoRa_rxMode();
  Serial.println("LoRa init succeeded.");


  /*Dump Register to check default config*/
  //LoRa.dumpRegisters(Serial);
}

/* Set flag if reception occured*/
void onReceive(int packetSize) 
{
  Serial.println("Packet received ");
  packageSize = packetSize;
  rxDoneFlag = true;
}

/* Set flag once transmission done*/
void onTxDone() 
{
  Serial.println("TX Done ");
}

#define MESSAGE_LEN 30
uint8_t lora_message[MESSAGE_LEN] = {0};

void loop() {

  while(1)
  {
    if (rxDoneFlag == true)
    {
      for (int i = 0; i < packageSize; i++)
      {
        lora_message[i] = LoRa.read();
      }


      Serial.print("Received packet: ");
      for (int i = 0; i < packageSize; i++)
      {
        Serial.print(lora_message[i], HEX);
        Serial.print(":");
      }
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
      
      rxDoneFlag = false;
    }
  }
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet");
    Serial.print(" Length: ");
    Serial.print(packetSize, DEC);
    Serial.print(" Payload: ");

    // read packet
    uint32_t receivedLen = 0;
    uint8_t rxByte = 0;
    while (LoRa.available()) 
    {
      rxByte = LoRa.read();
      Serial.print(rxByte, HEX);
      Serial.print(":");

      if(MESSAGE_LEN > receivedLen)
      {
        lora_message[receivedLen] = rxByte;
        receivedLen++;
      }
    }
    
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
