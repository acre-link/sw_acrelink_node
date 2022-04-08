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

#define LORA_MAX_LENGTH 100
#define LORA_NUM_OF_MESSAGES 3
struct LORA_RX_MESSAGE 
{
  volatile uint8_t rx_rssi; 
  volatile uint8_t rx_length;
  volatile uint8_t rx_data[LORA_MAX_LENGTH];
};
volatile uint8_t rxMessageIndex = 0;
volatile LORA_RX_MESSAGE rxMessages[LORA_NUM_OF_MESSAGES];



/* Set flag if reception occured*/
void onReceiveDone(int packetSize) 
{
if( false == rxDoneFlag )
{
   for(int i = 0; i < packetSize; i++)
  {
    rxMessageBuffer[i] = LoRa.read();
  }
  rxPacketSize = packetSize;
  rxRssi = LoRa.packetRssi();

  rxDoneFlag = true;
}
else
{
  /*Message buffer overrun. Can not handle new message*/
}
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

  
  //LoRa.onTxDone(onTransmittDone);
  LoRa.receive();
}

#define MAXNUMOFVALUES 20
struct DATA
{
   uint32_t device_id;
   uint8_t rssi;
   String recorded_at;
   uint16_t values[MAXNUMOFVALUES];  //Will be transmitted as HEX String to JSON
};

DATA package1, package2;

void loop() {
  if(rxDoneFlag == true)
  {
    Serial.print("Received something. Length: ");
    Serial.print(rxPacketSize, DEC);
    Serial.print(" Payload: "); 
    int i = 0;
    for(i = 0; i < rxPacketSize - 1; i++)
    {
      Serial.print(rxMessageBuffer[i], HEX);
      Serial.print(":");
    }
    Serial.print(rxMessageBuffer[i], HEX);
    
    Serial.print(" RSSI: ");
    Serial.println(rxRssi, DEC); 

    /*Decode and put into structure*/

    if(rxPacketSize >= 6) /*Only accept if package has at least one 16 bit sensor value*/
    {
      package1.device_id = rxMessageBuffer[0] << 24 | rxMessageBuffer[1] << 16 | rxMessageBuffer[2] << 8 | rxMessageBuffer[3];
      package1.rssi = rxRssi;
      package1.recorded_at = "20210328184901";

      int numberOfValues = (rxPacketSize - 4) / 2; 
      if (numberOfValues > MAXNUMOFVALUES)
      {
        numberOfValues = MAXNUMOFVALUES; //Limit maximum value count.
      }
      for ( int i = 0; i < numberOfValues; i++)
      {
        package1.values[i] = rxMessageBuffer[4 + i*2]<< 8 | rxMessageBuffer[5 + i*2];
      }      
    }
    rxDoneFlag = false;
  }
}
