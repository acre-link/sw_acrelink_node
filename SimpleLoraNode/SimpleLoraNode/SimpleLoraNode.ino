/*
  Simple LoRa Node
  Sends a message and goes to deep sleep after tx is done. 
  ESP32 WROOM uses a 40MHz crystal
  20210110
  by Maximilian Betz
 */

#define F_CPU 80000000L

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <time.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <esp_system.h>

const long frequency = 868E6;  // LoRa Frequency

const int misoPin = 19;
const int mosiPin = 23;
const int sckPin = 18;
const int csPin = 5;          // LoRa radio chip select
const int resetPin = 15;        // LoRa radio reset
const int irqPin = 4;          // change for your board; must be a hardware interrupt pin
const int oneWireEnablePin = 35;
const int oneWireDataPin1 = 25;
const int oneWireDataPin2 = 26;

const int spreadingFactor = 10;  //8 default
const int txPower = 14; //14 is the legal limit on 868.0 - 868.7
const int sleepTimeS = 10;
const int maximumAwakeTimeMs = 3000;
const uint8_t nodeGeneration = 1; //Used so that the gateway can distinguish between different nodes.

char printBuffer[100] = {0};  //Just some static memory for sprintf / print functions.

OneWire oneWire1(oneWireDataPin1);
DS18B20 sensor1(&oneWire1);

void getId(uint8_t *id)
{
  /* Get MAC address from WIFI and use for Node ID in Lora message.*/
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);

  id[0] = nodeGeneration;
  id[1] = baseMac[3];
  id[2] = baseMac[4];
  id[3] = baseMac[5];

  sprintf(printBuffer, "Node Id: %02X:%02X:%02X:%02X", id[0], id[1], id[2], id[3]);
  Serial.println(printBuffer);
}

/*Initialize hardware and configure Lora module*/
void setup() {
  setCpuFrequencyMhz(80); 
  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  Serial.println("LoRa Node woken up!");

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

  /*Enable OneWire MOSFET Power Supply*/
  pinMode(oneWireEnablePin, OUTPUT);
  digitalWrite(oneWireEnablePin, LOW); 

  sensor1.begin();
}

float get_temperature(void)
{
    /*First get Temperature*/
  int start = millis();
  int i = 0;
  float temperature = -273.15;
  
  sensor1.setResolution(10); //Low Resolution to speed up conversion time. Conversion time: 9=100ms, 10=180ms, 11=340ms, 12=658ms 
  sensor1.requestTemperatures();
  
  while (!sensor1.isConversionComplete())  // wait until sensor is ready. Attention never finishes if sensor is not available.
  {
    if (runEvery(100))
    {
      i++;
      Serial.println("Waiting for Temperature Conversion Done...");
    }
    
    if(i > 15)  //Should be sufficient even for 12 Bit resolution!
    {
      Serial.println("Aborted Waiting for Temperature Conversion Done...");
      break; // Do not drain battery and wait forever here!
    }
  }
  temperature = sensor1.getTempC();

  if((int)temperature  == DEVICE_DISCONNECTED)
  {
     Serial.println("No Temperaturesensor connected.");  //Somehow does not work. Better to check the address for 0!
  }
  else if((int)temperature == DEVICE_CRC_ERROR)
  {
     Serial.println("Temperaturesensor CRC error.");
  }

  Serial.print("Temperature Sensore Address: ");
  byte buffer[8] = {0,0,0,0,0,0,0,0};
  sensor1.getAddress(buffer);
  for (int i = 0; i < 8; i++)
  {
    Serial.print(buffer[i], HEX);
  }
  Serial.println();

 if ((buffer[0] == 0) && (buffer[1] == 0) && (buffer[2] == 0) && (buffer[3] == 0) && (buffer[4] == 0)) 
 {
   temperature = -273.15;  //Invalid temperature if there is no OneWire Sensor found on the Bus!
 }

  int duration = millis() - start;
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" Measurement duration: ");
  Serial.println(duration, DEC);

  return temperature;
}

#define MESSAGE_LEN 30
uint8_t lora_message[MESSAGE_LEN] = {0};
void loop() {
  /*
   * Message structure:   [id,id,id,id,temperaturehigh, temperaturelow]
   * 
  */

  /*Prepare and send the Lora message here*/
  float temperature = get_temperature();
  getId(lora_message);
  
  int16_t i16temperature = (int16_t) (temperature * 100); /*Scale temperature to 1/100 degrees centigrade*/ 
  lora_message[4] = (uint8_t)((i16temperature >> 8));
  lora_message[5] = (uint8_t)(i16temperature & 0xFF);
  uint8_t lora_message_length = 6;
  char printfbuf[50] = {0};
   
  Serial.print("Sending Message: ");
  for(int i = 0; i<lora_message_length; i++)
  {
    sprintf(printfbuf, "%02X:", lora_message[i]);
    Serial.print(printfbuf);
  }
  Serial.println("");
  
  LoRa.beginPacket();                   // start packet
  for(int i = 0; i < 6; i++)
  {
      LoRa.print(lora_message[i]);     // add payload
  }

  LoRa.endPacket(true);                 // finish packet and send it
  //LoRa_sendMessage(message, (uint8_t)MESSAGE_LEN);

  while(true)
  { 
    //Wait for tx Done and then go to sleep. 
    while(millis() < maximumAwakeTimeMs)
    {
      if (runEvery(100))
      {
        Serial.println("Waiting for Tx Done...");
      }
    }
    
     //Force going to sleep after 1 second. In case Lora does not finish sending. 
    Serial.println("Forcing going to sleep without waiting for Tx done. time_ms: ");
    Serial.println(millis(), DEC); 
    goToDeepSleep(); //Test what happens
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

/*For ASCII String messages*/
void LoRa_sendMessage(String &message) {
  Serial.print("SendingMessage time_ms: ");
  Serial.println(millis(), DEC);
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);             // add payload
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

void goToDeepSleep(void)
{
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

/* Gets called once LORA is done transmitting the message*/
void onTxDone() {
  Serial.print("TxDone time_ms: ");
  Serial.println(millis(), DEC);
  goToDeepSleep();
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
