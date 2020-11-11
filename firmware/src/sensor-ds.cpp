#ifdef SENS_DS
#include "protocol.h"
#include "sensors.h"
#include <OneWire.h>
#include <CayenneLPP.h>

//Schedule sensore measurement every this senconds
#define SENSOR_INTERVAL (1*30)//seconds

struct dssens {
  byte type_s;
  byte data[12];
  byte addr[8];
};

enum { 
    DSPIN = 3
};

static osjob_t sensjob;
static dssens sensor;
CayenneLPP lpp(51);
OneWire  ds( DSPIN );  // on pin 10 (a 4.7K resistor is necessary)


static float ds_readTemperature( struct dssens* ds_hdl ) {
    ds.reset();
    ds.select( ds_hdl->addr );
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    ds.reset();
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

static void sensors_update( osjob_t* j ) {

    float t = ds_readTemperature( &sensor );
    SENSORS_PRINT_F("Temperature: "); 
    SENSORS_PRINT(t);
    SENSORS_PRINT_F(" *C\n");
    lpp.reset();
    lpp.addTemperature(1, t);
    protocol_updateDataFrame( lpp.getBuffer(), lpp.getSize() );

    os_setTimedCallback( &sensjob, os_getTime() + sec2osticks(SENSOR_INTERVAL), sensors_update );
}

void sensors_init( ) {

    if ( !ds.search(sensor.addr)) {
        SENSORS_PRINT_F("No more addresses.\n");
        ds.reset_search();
        delay(250);
        return;
    }
    
    if (OneWire::crc8(sensor.addr, 7) != sensor.addr[7]) {
        SENSORS_PRINT_F("CRC is not valid!\n");
        return;
    }
  
    // the first ROM byte indicates which chip
    switch (sensor.addr[0]) {
      case 0x10:
        SENSORS_PRINT_F("  Chip = DS18S20\n");  // or old DS1820
        sensor.type_s = 1;
        break;
      case 0x28:
        SENSORS_PRINT_F("  Chip = DS18B20\n");
        sensor.type_s = 0;
        break;
      case 0x22:
        SENSORS_PRINT_F("  Chip = DS1822\n");
        sensor.type_s = 0;
        break;
      default:
        SENSORS_PRINT_F("Device is not a DS18x20 family device.\n");
        return;
    } 
    // Schedule next transmission
    os_setCallback( &sensjob, sensors_update );
}

#endif