
#include <OneWire.h>
#include "ds.h"

enum { 
    DSPIN = 3
};
  
OneWire  ds( DSPIN );  // on pin 10 (a 4.7K resistor is necessary)



void ds_init( struct dssens* ds_hdl ) {
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


float ds_readTemperature( struct dssens* ds_hdl ) {
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



