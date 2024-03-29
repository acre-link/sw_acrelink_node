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

// Good reference for the following: https://www.electroniclinic.com/esp32-wroom-32d-pinout-features-and-specifications/
const int misoPin = 19;  //PIN31  GPIO19      V1_HW:  SPI_MISO
const int mosiPin = 23;  //PIN37   GPIO23     V1-HW:  SPI_MOSI
const int sckPin = 18;   //PIN30   GPIO18     V1-HW: SPI_SCK
const int csPin = 5;     //PIN29  GPIO5     // LoRa radio chip select
const int resetPin = 15;  //PIN23  GPIO15   // LoRa radio reset
const int irqPin = 4;  //PIN26  GPIO4       // change for your board; must be a hardware interrupt pin
const int voltageSenseEnablePin = 34;   //PIN6  GPIO34    V1-HW: VDD_SENSE_MOSFET_SWITCH
const int oneWireEnablePin = 35;  //PIN7  GPIO35    V1-HW: VDD_SENSOR_MOSFET_SWITCH
const int oneWireDataPin1 = 25;  //PIN10  GPIO25    V1-HW: ONEWIRE1_DATA


//Test these pins for GPIO and Analog Functionality:
const int analogInput1 = 14;  //PIN13  GPIO14  ADC2_CH6     V1-HW: UART1_RX   //ADC input works: 1.6V ~ 1780(12bit value, attn 11db)
const int analogInput2 = 13;  //PIN16 GPIO13 ADC2_CH4      V1-HW: UART1_TX      //ADC input works 

const int voltageSensePin = 26; //PIN11 GPIO26;     V1-HW: ONEWIRE2_DATA    //ADC input works     Voltage
const int gps_mosfet_switch = 22;   //PIN36  GPIO22;   V1-HW: I2C_SCL     //GPIO controll works.

const int whateverA = 21;  //PIN33 GPIO21       V1-HW: I2C_SDA     // GPIO out works
const int whateverB = 16;  //PIN27 GPIO16       V1-HW: UART2_RX     // GPIO out works
const int whateverC = 17;  //PIN28 GPIO17       V1-HW: UART2_TX    //GPIO out works
const int buck_5v_en = 27;  //PIN12 GPIO27       V1-HW: GPS MOSFET Switch    //GPIO out works
const int whateverE = 32;  //PIN8 GPIO32        V1-HW: XTAL32_XP
const int whateverF = 33;  //PIN9 GPIO33        V1-HW: XTAL32_XN

// Other defines:
const int spreadingFactor = 10;  //8 default
const int txPower = 14; //14 is the legal limit on 868.0 - 868.7
const int sleepTimeS = 10;
const int maximumAwakeTimeMs = 30000;
const uint8_t nodeGeneration = 1; //Used so that the gateway can distinguish between different nodes.


/* Global objects */
char printBuffer[100] = {0};  //Just some static memory for sprintf / print functions.
bool txDoneFlag = false;
bool rxDoneFlag = false;
OneWire oneWire1(oneWireDataPin1);
DS18B20 sensor1(&oneWire1);

RTC_DATA_ATTR uint16_t batteryVoltage = 0;  //Survives deep sleep. 

/*#############################################################################*/
/*#############################################################################*/
/* Helper functions */

/* Get MAC address from WIFI and use for Node ID in Lora message.*/ 
void getId(uint8_t *id)
{
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);

  id[0] = nodeGeneration;
  id[1] = baseMac[3];
  id[2] = baseMac[4];
  id[3] = baseMac[5];

  sprintf(printBuffer, "Node Id: %02X:%02X:%02X:%02X", id[0], id[1], id[2], id[3]);
  Serial.println(printBuffer);
}

/* Get temperature from onewire sensor.*/ 
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

  Serial.print("Temperature Sensor Address: ");
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

/* Set LoRa to rx mode.  onReceive will be triggerd after a reception.*/
void LoRa_rxMode()
{
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

/* Set LoRa to tx mode.  onTxDone will be triggered after the transmission. */
void LoRa_txMode()
{
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

/* Set flag if reception occured*/
void onReceive(int packetSize) 
{
  rxDoneFlag = true;
}

/* Set flag once transmission done*/
void onTxDone() 
{
  txDoneFlag = true;
}

/* Configure pins and pheripheral for very low sleep current and got to sleep.*/
void goToDeepSleep(uint8_t delay_offset)
{
  uint32_t baseSleepTime = 1000000 * sleepTimeS;
  uint32_t individualSleepTime = baseSleepTime + 300000 * delay_offset;  // Some pseudo random offset to the sleep time.

  individualSleepTime = 12000000; //Debug send more often at 1 /10s
  Serial.print("Sleeping for: ");
  Serial.print((individualSleepTime / 1000000), DEC);
  Serial.println("s");
  LoRa.sleep();

  /*Configure pins for low sleep current.*/
  pinMode(misoPin, INPUT_PULLUP);
  pinMode(mosiPin, INPUT_PULLUP);
  pinMode(sckPin, INPUT_PULLUP);
  pinMode(csPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP); 

  digitalWrite(buck_5v_en, HIGH);  //Switch off 5V Buck
  
  esp_sleep_enable_timer_wakeup(individualSleepTime); 
  esp_deep_sleep_start();
}

/* Must be called cyclically and returns true once interval expired. */
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


/*#############################################################################*/
/*#############################################################################*/
/* Main application code */
#define MESSAGE_LEN 30
/*
 * Message structure:   [id,id,id,id,temperaturehigh, temperaturelow]
 */
uint8_t lora_message[MESSAGE_LEN] = {0};

/* Initialize hardware and configure pheripharals. */
void setup() 
{
  setCpuFrequencyMhz(80); 
  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  Serial.println("LoRa Node woken up!");

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) 
  {
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
  /*LoRa should be initialized here and in sleep mode*/
  
  /*Enable OneWire MOSFET Power Supply*/
  //pinMode(oneWireEnablePin, OUTPUT);
  //digitalWrite(oneWireEnablePin, LOW); 
  sensor1.begin();

  //pinMode(voltageSenseEnablePin, OUTPUT);
  //digitalWrite(voltageSenseEnablePin, LOW);


 
  pinMode(analogInput1, INPUT); 
  adcAttachPin(analogInput1);
  pinMode(analogInput2, INPUT); 
  adcAttachPin(analogInput2);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(voltageSensePin, INPUT); 
  adcAttachPin(voltageSensePin);


  pinMode(buck_5v_en, OUTPUT);
  digitalWrite(buck_5v_en, LOW);  //Switch on 5V Buck

}


/* Read temperature, id, assemble package and transmit it.*/
void loop() 
{
  /*Prepare and send the Lora message here*/
  float temperature = get_temperature();
  getId(lora_message);
  
  int16_t i16temperature = (int16_t) (temperature * 100); /*Scale temperature to 1/100 degrees centigrade*/ 
  lora_message[4] = (uint8_t)((i16temperature >> 8));
  lora_message[5] = (uint8_t)(i16temperature & 0xFF);

  /*Add humitiy*/
  lora_message[6] = 0xFF;   /*TODO: add sensor*/
  lora_message[7] = 0xFF;   
  
  /* Add digital input bits*/
  lora_message[8] = 0xAA;  /*TODO: add digital inputs and read bits*/
  lora_message[9] = 0xAA; 

  /*Add own battery voltage.*/
  lora_message[10] = (uint8_t)((batteryVoltage >> 8));
  lora_message[11] = (uint8_t)(batteryVoltage & 0xFF);
  
  uint8_t lora_message_length = 12;
  char printfbuf[50] = {0};
   
  Serial.print("Sending Message: ");
  for(int i = 0; i < lora_message_length; i++)
  {
    sprintf(printfbuf, "%02X:", lora_message[i]);
    Serial.print(printfbuf);
  }
  Serial.println("");
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.write(lora_message, lora_message_length);
  LoRa.endPacket(true);                 // finish packet and send it
  batteryVoltage = batteryVoltage + 1;   //TODO: just use this as a counter.   How to read the voltage from Sensor_VP input?
  int i  = 0;
  while(true)
  {     
    //Wait for tx Done and then go to sleep. 
    while(true)
    {
      
     if (runEvery(100))
     {
      //Serial.println("Waiting for Tx Done...");  

     
      Serial.print("Input1: ");
      Serial.println(analogRead(analogInput1), DEC); 

      Serial.print("Input2: ");
      Serial.println(analogRead(analogInput2), DEC); 

      
      

     }
      
     if (millis() >= maximumAwakeTimeMs)
     {
          Serial.println("Forcing going to sleep. Tx not done! ms: ");
          break;
     }

     if(true == txDoneFlag)
     {
        //Serial.print("TxDone ms: ");
        //break;
     }
   }
   Serial.println(millis(), DEC); 
   goToDeepSleep(lora_message[3]); // Use the last byte of the MAC Address to do some individual sleep interval to reduce canceling out chances.
 }
}
