/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
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
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT_U.h"
#include <CayenneLPP.h>
#include <OneWire.h>
#include <ArduinoUniqueID.h>
#include "credentials.h"

#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
void do_send(osjob_t* j);

static uint8_t mydata[] = { 0,0,0,0,0,0,0,0};
static osjob_t sendjob;

OneWire  ds(3);  // on pin 10 (a 4.7K resistor is necessary)

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

CayenneLPP lpp(51);


// Pin mapping
//const lmic_pinmap lmic_pins = {
//    .nss = 6,
//    .rxtx = LMIC_UNUSED_PIN,
//    .rst = 5,
//    .dio = {2, 3, 4},
//};

//const lmic_pinmap lmic_pins = {
//    .nss = 8,
//    .rxtx = LMIC_UNUSED_PIN,
//    .rst = 4,
//    .dio = {3, 6, 11},
//};

//Leonardo
//const lmic_pinmap lmic_pins = {
//    .nss = 8,
//    .rxtx = LMIC_UNUSED_PIN,
//    .rst = 4,
//    .dio = {7, 6, LMIC_UNUSED_PIN},
//};

//Maping for IOTMCU arduino pro MINI
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 7, LMIC_UNUSED_PIN},
};

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
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

float h = 0;
float t = 0;


void do_send(osjob_t* j){
    
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare data transmission at the next possible time.
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        //LMIC_setTxData2(1,(uint8_t*)&t,sizeof(t),0);
        LMIC_setTxData2(1, mydata, strlen((char*) mydata), 0);
    }
}

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  
    Serial.begin(115200);
    //while (!Serial);
    Serial.println(F("Starting"));

    dht.begin();


Serial.println("Test");


    // LMIC init
    os_init();

    Serial.println("Test2");
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    Serial.println("Test3");

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
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
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
    //do_send(&sendjob);
}


int i = 30;

void loop() {
  os_runloop_once();

  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  #if 0
  Serial.print("ROM =");
  for( byte i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  #endif

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 


  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end



delay(1000);


  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  //fahrenheit = celsius * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheit);
  //Serial.println(" Fahrenheit");





  

if (i == 30)
{

    // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = 50.0;//dht.readHumidity();
  // Read temperature as Celsius
  t = celsius;//dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");    
  }
  else
  {
    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.println(" *C ");
    Serial.println("Do Send");


  // Esto funciona perfecto ------------------------------------------------------------
//    char json_data[60];
//  
//    float temp = dht.readTemperature();
//    float humidity = dht.readHumidity();
//    
//    int temp_decimal1 = (int)temp;
//    temp -= temp_decimal1;
//    temp *= 100;
//    int temp_decimal2 = (int)temp;
//    int hum_decimal1 = (int)humidity;
//    humidity -= hum_decimal1;
//    humidity *= 100;
//    int hum_decimal2 = (int)humidity;
//    snprintf(json_data, 60, "{\"T\":%d.%d,\"H\":%d.%d}", temp_decimal1, temp_decimal2, hum_decimal1, hum_decimal2);
//    Serial.println(json_data);
//    //return LoRa_send(19, (uint8_t*) json_data, strlen(json_data));
//
//  if (LMIC.opmode & OP_TXRXPEND) {
//      Serial.println(F("OP_TXRXPEND, not sending"));
//  } else {
//      // Prepare data transmission at the next possible time.
//      //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
//      //LMIC_setTxData2(1,(uint8_t*)&t,sizeof(t),0);
//      LMIC_setTxData2(2, (uint8_t*) json_data, strlen(json_data), 0);
//  }
// --------------------------------------------------------------

// Con cayenne ----------------------------------------------------------
  lpp.reset();
  lpp.addTemperature(1, t);
  lpp.addRelativeHumidity(2, h);
  //lpp.addGPS(3, 52.37365, 4.88650, 2);

  // Send it off
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare data transmission at the next possible time.
      //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
      //LMIC_setTxData2(1,(uint8_t*)&t,sizeof(t),0);
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
  }

  // --> Esto es usando la librería de ttn --> ttn.sendBytes(lpp.getBuffer(), lpp.getSize());
  
// Fin Cayenne


//  float h_float = dht.readHumidity();
//  float t_float = dht.readTemperature(); 
//  
//  int h2 = (int)h;
//  int t2 = (int)t;
//
//  int batt = 0;  // readVCC returns  mVolt need just 100mVolt steps
//  byte batvalue = (byte)batt;
// 
//  byte data[4];
//  data[0] = batvalue;  
//  data[1] = h2 & 0xFF;
//  data[2] = t2 & 0xFF;
//  data[3] = t2 & 0xFF; 
//
//  if (LMIC.opmode & OP_TXRXPEND) {
//      Serial.println(F("OP_TXRXPEND, not sending"));
//  } else {
//      // Prepare data transmission at the next possible time.
//      //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
//      //LMIC_setTxData2(1,(uint8_t*)&t,sizeof(t),0);
//      LMIC_setTxData2(1, data, sizeof(data), 0);
//  }
  
//    dtostrf(t,5,2,(char*)mydata);
//    do_send(&sendjob);
    
  }
  i = 0;
}
else
{  
  i++;  
  //Serial.print(" - " + i);
}



 
}