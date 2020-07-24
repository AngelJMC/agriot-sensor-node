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
#include "AsyncSerialLib.h"
#include "credentials.h"
#include "avr/wdt.h"
#include <EEPROM.h>

#define EOL            '\n'

enum loglevel{
    ECHO = 0,
    DISABLE = 0,
    DEBUG,
    WARNING,
    ERROR
};

struct cfg{
    uint8_t nwkskey[16];
    uint8_t appskey[16];
    uint32_t devaddr;
};

struct cfg cfg;

void param_load(  );

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
void do_send(osjob_t* j);

static osjob_t sendjob;

#if SENS_DS18B20
  enum { 
    DSPIN = 3
  };
  OneWire  ds( DSPIN );  // on pin 10 (a 4.7K resistor is necessary)
#elif SENS_DHT22
  enum {
    DHTPIN = 3,     // what pin we're connected to
    DHTTYPE = DHT22   // DHT 22  (AM2302)
  };
  DHT dht( DHTPIN, DHTTYPE );
#endif 



CayenneLPP lpp(51);

//Maping for IOTMCU arduino pro MINI
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 7, LMIC_UNUSED_PIN},
};

static int i = SAMPLING_TIME;
static float h = 0;
static float t = 0;

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
            digitalWrite( A0, true );
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



#if SENS_DS18B20 
struct dssens {
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
};

static dssens ds_hdl;

static void ds_init( struct dssens* ds_hdl ) {
    if ( !ds.search(ds_hdl->addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
      return;
    }
    
    if (OneWire::crc8(ds_hdl->addr, 7) != ds_hdl->addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
  
    // the first ROM byte indicates which chip
    switch (ds_hdl->addr[0]) {
      case 0x10:
        Serial.println("  Chip = DS18S20");  // or old DS1820
        ds_hdl->type_s = 1;
        break;
      case 0x28:
        Serial.println("  Chip = DS18B20");
        ds_hdl->type_s = 0;
        break;
      case 0x22:
        Serial.println("  Chip = DS1822");
        ds_hdl->type_s = 0;
        break;
      default:
        Serial.println("Device is not a DS18x20 family device.");
        return;
    } 
}

static float ds_readTemperature( struct dssens* ds_hdl ) {
    ds.reset();
    ds.select( ds_hdl->addr );
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);

    ds_hdl->present = ds.reset();
    ds.select( ds_hdl->addr );    
    ds.write( 0xBE );         // Read Scratchpad

    for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
      ds_hdl->data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = ( ds_hdl->data[1] << 8) | ds_hdl->data[0];
    if ( ds_hdl->type_s ) {
      raw = raw << 3; // 9 bit resolution default
      if ( ds_hdl->data[7] == 0x10 ) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - ds_hdl->data[6];
      }
    } else {
      byte cfg = ( ds_hdl->data[4] & 0x60 );
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    return (float)raw / 16.0;
}
#endif

void do_send( osjob_t* j ) {
    digitalWrite( A0, false );
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare data transmission at the next possible time.
        //LMIC_setTxData2(1, mydata, strlen((char*) mydata), 0);
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    }
}


int const dataLength = 40;
byte data[dataLength];

enum resp{
  NOK   = -1,
  NOTHING = 0,
  OK      = 1
};

typedef struct ack{
  enum resp rp;
  char type;
}ack_t;

static int const eeadrInfo = 0; 

char cmd[40];
char txbuff[64];

enum loglevel verbose = DISABLE;


uint16_t sensticks = 0;
int incomingByte = 0;

void sendresponse( ack_t *ack);
ack_t parseCommands(AsyncSerial &serial);
void printdebug( const char * msg, int errlevel );

AsyncSerial asyncSerial(data, dataLength,
	[](AsyncSerial& sender) { ack_t ack = parseCommands( sender ); sendresponse( &ack ); }
);


void setup() {
  
    wdt_disable();
    Serial.begin(9600);
    Serial.println(F("Starting"));
    param_load(  );

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    LMIC_setSession (0x1, cfg.devaddr, cfg.nwkskey, cfg.appskey );


    #if defined( CFG_eu868 )
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
    #elif defined( CFG_us915 )
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode( 0 );

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow( DR_SF7, 14 );

    asyncSerial.FinishChar = NEW_LINE;
    asyncSerial.IgnoreChar = CARRIAGE_RETURN;

#if SENS_DS18B20
      ds_init( &ds_hdl );
#elif SENS_DHT22
      dht.begin();
#endif

    do_send( &sendjob );
    pinMode( A1, INPUT ); 
    pinMode( A0, OUTPUT );

    //wdt_enable(WDTO_8S);

}


void loop() {
  
  //wdt_reset();
  asyncSerial.AsyncRecieve();

  os_runloop_once();
  ++sensticks;
  if( sensticks > 40000 ){
    sensticks = 0;
    if( !digitalRead( A1 ) ) do_send( &sendjob );
    i++;

    if ( i >= SAMPLING_TIME ) {
        i = 0;
        
  #if SENS_DS18B20
        t = ds_readTemperature( &ds_hdl );
        Serial.print("Temperature: "); 
        Serial.print(t);
        Serial.println(" *C ");
        Serial.println("Do Send");
        lpp.reset();
        lpp.addTemperature(1, t);
  #elif SENS_DHT22
        t = dht.readTemperature( ); // Read temperature as Celsius
        h = dht.readHumidity( );
        Serial.print("Temperature: "); 
        Serial.print(t);
        Serial.print(" *C ");
        Serial.print("Humidity: "); 
        Serial.print(h);
        Serial.print(" %\t");
        Serial.println("Do Send");
        lpp.reset();
        lpp.addTemperature(1, t);
        lpp.addRelativeHumidity(2, h);
  #endif
  }
      
  } 

}






int getNum(char ch)
{
    int num=0;
    if( ch>='0' && ch<='9' ) {
        num=ch-0x30;
    }
    else {
        switch( ch ) {
            case 'A': case 'a': num=10; break;
            case 'B': case 'b': num=11; break;
            case 'C': case 'c': num=12; break;
            case 'D': case 'd': num=13; break;
            case 'E': case 'e': num=14; break;
            case 'F': case 'f': num=15; break;
            default: num=0;
        }
    }
    return num;
}

uint8_t hex2int( char hex[])
{
    return getNum(hex[0])*16 + getNum(hex[1]);
}

void printdebug( const char * msg, int errlevel ) {

    char header[4];
    sprintf( header, "!%d,", errlevel );
    Serial.print( header );
    Serial.print( msg );
    Serial.print( EOL );
}


#if 1

ack_t parseCommands(AsyncSerial &serial) {

  ack_t ack = { .rp = NOK, .type = '\0' };
  memcpy(&cmd, serial.GetContent(), serial.GetContentLength());

  if ( ECHO )  //VERBOSE ECHO
    printdebug( cmd, DEBUG );
  switch ( cmd[0] ) {
    case 'N':
      {
          if( strlen(cmd) != 34 ) 
              return ack;

          for( int i = 2; i < 34; i = i +2) {
                  char cbyte[3]="";
                  cbyte[0] = cmd[i];
                  cbyte[1] = cmd[i+1];
                  cfg.nwkskey[i/2 - 1] = hex2int(cbyte);
          }
          EEPROM.put( eeadrInfo, cfg );
      }
      break;
    case 'A':
      {
          if( strlen(cmd) != 34 ) 
              return ack;

          for( int i = 2; i < 34; i = i +2) {
                  char cbyte[3]="";
                  cbyte[0] = cmd[i];
                  cbyte[1] = cmd[i+1];
                  cfg.appskey[i/2 - 1] = hex2int(cbyte);
          }
          EEPROM.put( eeadrInfo, cfg );
      }
      break;
    case 'D':
          {
              if( strlen(cmd) != 10 ) 
                  return ack;

              uint32_t x = 0;
              for( int i = 2; i < 10; ++i) 
                    x = x*16 + getNum( cmd[i] );
          
              cfg.devaddr = x;
              EEPROM.put( eeadrInfo, cfg );
          }
      break;
    case 'I':
        Serial.print("NetKey: ");
        for( int i = 0; i < 16; ++i) { 
          Serial.print(cfg.nwkskey[i], HEX );
        }
        Serial.print("\nAppKey: ");
        for( int i = 0; i < 16; ++i) { 
          Serial.print(cfg.appskey[i], HEX );
        }
      Serial.print(F("\nSensor Addr: "));
      Serial.println( cfg.devaddr, HEX );
      //Serial.print( EOL );
      break;

    default:
      break;
  }
    

    ack.rp = OK;
    #if 0
    if ( verbose >= DEBUG ) { //VERBOSE
      sprintf( txbuff, "%s %d, %d, %d, %d", "RACE CONFIG: ",
                                    cfg.startline,
                                    cfg.nlap,
                                    cfg.nrepeat,
                                    cfg.finishline );
      printdebug( txbuff, DEBUG );
    }
    #endif

  return ack;
}

void sendresponse( ack_t *ack) {

  if( ack->rp == OK ) { 
    Serial.print( ack->type );
    Serial.print("OK");
    Serial.print( EOL );
  }
  else if( ack->rp == NOK ) {
    Serial.print( ack->type );
    Serial.print("NOK");
    Serial.print( EOL );
  }
  memset( &cmd, '\0' , sizeof(cmd) );
}
#endif

#define CFG_VER 1

void param_setdefault( struct cfg* cfg ) { 
  memcpy_P(cfg->appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(cfg->nwkskey, NWKSKEY, sizeof(NWKSKEY));
  cfg->devaddr = DEVADDR;
  
}

void param_load(  ) {
    int cfgversion=0;
    int eeAdress = eeadrInfo;
    EEPROM.get( eeAdress, cfg );
    eeAdress += sizeof( struct cfg );
    EEPROM.get( eeAdress, cfgversion );
    
    if ( cfgversion != CFG_VER ) {
      param_setdefault( &cfg );
      eeAdress = 0;
      EEPROM.put( eeAdress, cfg );
      eeAdress += sizeof( struct cfg );
      EEPROM.put( eeAdress, CFG_VER );
      Serial.print(F("LOAD DEFAULT\n"));
    }
}