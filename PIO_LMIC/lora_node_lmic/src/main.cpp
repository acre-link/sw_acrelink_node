#include <Arduino.h>
#include <CayenneLPP.h>

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//

//#ifdef COMPILE_REGRESSION_TEST
//#define FILLMEIN 0
//#else
//#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
//#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
//#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0xE4, 0xC0, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x6C, 0xAC, 0x3C, 0xB3, 0x6E, 0x85, 0xDE, 0x7B, 0x1A, 0x9B, 0xA5, 0x31, 0x1A, 0xD2, 0x02, 0x60};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }


// Provide a RTC memory space to store and restore session information during deep sleep.
RTC_DATA_ATTR static lmic_t RTC_LMIC;
bool GOTO_DEEPSLEEP = false;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 15,
    .dio = {4, 32, 34},
};

// Pin definitions
const int POWER_EN_PIN = 27;
const int VDD_SENSE_PIN = 13;
const int ANALOG_SENSE1_PIN = 14;
const int ANALOG_SENSE2_PIN = 26;
const int LED_SIGNAL_PIN = 33;

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(nwkKey[i]);
            }
            Serial.println();
            Serial.print("rxDelay: ");
            Serial.println(LMIC.rxDelay, DEC);
        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_RFU1:
    ||     Serial.println(F("EV_RFU1"));
    ||     break;
    */
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        GOTO_DEEPSLEEP = true; // Start deep sleep and schedule next TX after wakeup
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_SCAN_FOUND:
    ||    Serial.println(F("EV_SCAN_FOUND"));
    ||    break;
    */
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;

    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void do_send_cayenne(float voltage, float humidity, float temperature)
{
    CayenneLPP lpp(51);
    lpp.reset();
    lpp.addAnalogInput(1, voltage);  //there is no working voltage channel with ttn. 
    lpp.addRelativeHumidity(2, humidity);
    lpp.addTemperature(3, temperature);

    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        LMIC_setTxData2_strict(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.print(F("Packet queued, LMIC.opmode: "));
        Serial.println(LMIC.opmode, HEX);

        Serial.print(F("os_getTime: "));
        Serial.println(os_getTime(), DEC);

        Serial.print(F("millis(): "));
        Serial.println(millis(), DEC);
        
        Serial.print(F("RTC_LMIC.globalDutyAvail: "));
        Serial.println(RTC_LMIC.globalDutyAvail, DEC);

        Serial.print(F("LMIC.osjob.deadline: "));
        Serial.println(LMIC.osjob.deadline, DEC);
    }

}

void SaveLMICToRTC(int deepsleep_sec)
{
    // Copy instance memory to deep sleep rtc memory.
    RTC_LMIC = LMIC;
    // EU Like Bands

    // System time is resetted after sleep. So we need to calculate the dutycycle with a resetted system time
    unsigned long now = os_getTime(); //does this change fixes the problem. millis();
#if defined(CFG_LMIC_EU_like)
    for (int i = 0; i < MAX_BANDS; i++)
    {
        ostime_t correctedAvail = RTC_LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
        if (correctedAvail < 0)
        {
            correctedAvail = 0;
        }
        RTC_LMIC.bands[i].avail = correctedAvail;
    }
    RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (RTC_LMIC.globalDutyAvail < 0)
    {
        RTC_LMIC.globalDutyAvail = 0;
    }
#else
    Serial.println("No DutyCycle recalculation function!")
#endif
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Setup start.");

    pinMode(POWER_EN_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(POWER_EN_PIN, LOW); // Enable 5V Buck converter.

    pinMode(LED_SIGNAL_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(LED_SIGNAL_PIN, LOW); // Enable LED

    pinMode(VDD_SENSE_PIN, INPUT);
    pinMode(ANALOG_SENSE1_PIN, INPUT);
    pinMode(ANALOG_SENSE2_PIN, INPUT);

    adcAttachPin(VDD_SENSE_PIN);
    adcAttachPin(ANALOG_SENSE1_PIN);
    adcAttachPin(ANALOG_SENSE2_PIN);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Load the LoRa information from RTC
    if (RTC_LMIC.seqnoUp != 0)
    {
        Serial.println("Waking up from deepsleep.");
        Serial.print("Up sequence number: ");
        Serial.println(RTC_LMIC.seqnoUp);
        LMIC = RTC_LMIC;
        LMIC.opmode = OP_NONE; // reset state Works but why do we need this?

        LMIC.globalDutyAvail =  os_getTime(); //millis() + 10; //allow sending a packet in 10ms. Could violate duty cycle limitation.


        for (int i = 0; i<MAX_BANDS; i++)
        {
            LMIC.bands[i].avail = os_getTime();  // Allow sending on all channels right away.
        }
        
    }
    else
    {
        Serial.println(F("Cold start."));
    }

    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);  //60

    LMIC_setDrTxpow(DR_SF12, 14);  //14 or 20?

    // Start job (sending automatically starts OTAA too)
    //do_send(&sendjob);

    // 0 == external power,  1 - 254 == battery level,  255 == no information possible
    LMIC_setBatteryLevel(100);  //TODO: adjuste to battery voltage
}

int32_t timeTillJob = 0;
int i = 0;
//float vdd_voltage = 0.0f;
//float humidity = 0.0f;
//float temperature = 0.0f;
//bool started = false;
void loop()
{
    static bool started = false;
    static float vdd_voltage = 0.0f;
    static float humidity_sense = 0.0f;
    static float temperature_sense = 0.0f;

    os_runloop_once();

    if (i < 20)
    {
        i++;

        // Humidity 1.79V == unter wasser  == 2127 ADC Wert       2 V == 2377 == 100%
        // Bat :; 3.32V == 905 ADC Wert      3.32/905  *ADC value
        // Temp 1130 == 1V == 22.6°C     //-40 == 0V   +80  == 2V   == 120Grad Range

        vdd_voltage = (float)analogRead(VDD_SENSE_PIN);
        vdd_voltage = vdd_voltage * 0.003668508f; // experimentally calculated

        humidity_sense = (float)analogRead(ANALOG_SENSE1_PIN);
        humidity_sense = humidity_sense / 23.770f; // experimentally calculated

        temperature_sense = (float)analogRead(ANALOG_SENSE2_PIN);
        temperature_sense = (temperature_sense / 1130.0f * 60.0f) - 40.0f;

        Serial.print("VDD Sense: ");
        Serial.print(vdd_voltage * 1000.0, DEC);

        Serial.print(" Humidity: ");
        Serial.print(humidity_sense * 1000.0, DEC);

        Serial.print(" Temperature: ");
        Serial.println(temperature_sense * 1000.0, DEC);
    }
    else
    {
        if(false == started)
        {
            started = true;
            do_send_cayenne(vdd_voltage, humidity_sense, temperature_sense);
        }        
    }

    int seconds = 60;
    if (true == GOTO_DEEPSLEEP)
    {
        SaveLMICToRTC(seconds);
        Serial.println("Going to deepsleep. ");
        esp_sleep_enable_timer_wakeup(seconds * 1000000);
        esp_deep_sleep_start();
    }
}