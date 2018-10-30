/* 
* TheThingsNetwork Payload functions : 
function Decoder(bytes, port)
{
var decoded = {};

if (port === 1) {
decoded.batt = (bytes[0] +250) / 100.0;
if (decoded.batt === 0)
delete decoded.batt;
}

if (port === 2) {
decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
decoded.lat = (decoded.lat / 16777215.0 * 180) – 90;
decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
decoded.lon = (decoded.lon / 16777215.0 * 360) – 180;
}
return decoded;
} 
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "LowPower.h"

//ADXL
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

//LowPower
bool next = false;

//TinyGPS
TinyGPSPlus gps;

//SoftwareSerial (RX,TX)
SoftwareSerial ss(8, 9);

//LMIC
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const u4_t DEVADDR = 0x12345678; // <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Schedule TX every this many seconds
const unsigned TX_INTERVAL = 24; //multiple of 8

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {4, 5, 7},
};

static osjob_t sendjob;

//Battery
int BattValue = 0;

//Definition of Pins
int BattOut = A0;   //Voltage Divider INPUT
int GPSVCC = 3;     //GPS Sensor OUTPUT
int gpsvcc3 = 15;     //GPS Sensor OUTPUT
int gpsvcc1 = 16;     //GPS Sensor OUTPUT
int gpsvcc2 = 17;     //GPS Sensor OUTPUT

bool activity= false;
int alive = 0;
uint8_t measurement[1];
uint8_t coords[6];
uint32_t LatitudeBinary, LongitudeBinary;

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
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
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              activity = false; //deactivate sending of GPS coordinates
              digitalWrite(gpsvcc1, LOW);
              digitalWrite(gpsvcc2, LOW);
              digitalWrite(gpsvcc3, LOW);
            }
            // Schedule next transmission
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            next = true; 
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){


  LMIC_setDrTxpow(DR_SF7,14);
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

          BattValue = averageAnalogRead(BattOut);
          int battVoltage = ( 3.36 / 1024 * BattValue * 2 * 100 ) -250; //Noch 250 abziehen, später im Decoder +250 /100
          Serial.println(battVoltage);
          measurement[0] = battVoltage;
        
        LMIC_setTxData2(1, (uint8_t*) measurement, sizeof(measurement), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void do_sendg(osjob_t* j){


  LMIC_setDrTxpow(DR_SF7,14);
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        get_coords();
        LMIC_setTxData2(2, (uint8_t*) coords, sizeof(coords), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

void get_coords() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float flat, flon;
  unsigned long age;
 
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss.available()) {
      char c = ss.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      gps.encode(c);
    }
  }
 
  if (gps.location.isValid()) {
    LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
    LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
  
    coords[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    coords[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    coords[2] = LatitudeBinary & 0xFF;
  
    coords[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    coords[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    coords[5] = LongitudeBinary & 0xFF;
  }
}

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup(){
  
  Serial.begin(9600);                 // Start the serial terminal
  Serial.println("Starting");
  ss.begin(9600);

  pinMode(BattOut, INPUT);
  pinMode(gpsvcc1, OUTPUT);
  pinMode(gpsvcc2, OUTPUT);
  pinMode(gpsvcc3, OUTPUT);

  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(16);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity

//  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
//                                      // Default: Set to 1
//                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
   
  adxl.setActivityXYZ(1, 1, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
//  adxl.setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
//  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
//  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?
//
//  adxl.setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
// 
//  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
//  adxl.setTapThreshold(50);           // 62.5 mg per increment
//  adxl.setTapDuration(15);            // 625 μs per increment
//  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
//  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
// 
//  // Set values for what is considered FREE FALL (0-255)
//  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
//  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(0);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  #if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
  #elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);

  // Start job
  do_send(&sendjob);
}

/****************** MAIN CODE ******************/
/*     Accelerometer Readings and Interrupt    */
void loop(){

  extern volatile unsigned long timer0_overflow_count;
  
  if (next == false) {

    os_runloop_once();

  } else {

    int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
    Serial.flush(); // give the serial print chance to complete
    byte savedPCICR = PCICR;
    PCICR = 0;  // Disable all pin change interrupts
    for (int i=0; i<sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

      // LMIC uses micros() to keep track of the duty cycle, so
      // hack timer0_overflow for a rude adjustment:
      cli();
      timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
      sei();
    }
    PCICR = savedPCICR;  // Restore any pin change interrupts that were disabled.
    alive = alive + 1;
    ADXL_ISR();
    // Start job
    if (activity == true){
      next = false;
      do_sendg(&sendjob);
    } else if (alive >= 35){
      next = false;
      alive = 0;
      Serial.println(F("Keep Alive"));
      do_send(&sendjob);
    }
  }
}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();

  // Activity
  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
    Serial.println("*** ACTIVITY ***"); 
    activity = true;
    digitalWrite(gpsvcc1, HIGH);
    digitalWrite(gpsvcc2, HIGH);
    digitalWrite(gpsvcc3, HIGH);
  }
  
}

