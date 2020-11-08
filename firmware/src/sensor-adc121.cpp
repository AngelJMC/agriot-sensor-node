#ifdef SENS_SOIL_MOISTURE_ADC
#include "protocol.h"
#include <Wire.h>
#include "sensors.h"
#include <CayenneLPP.h>

//Schedule sensore measurement every this senconds
#define SENSOR_INTERVAL (1*30)//seconds


static osjob_t sensjob;
CayenneLPP lpp(51);

enum {
    REG_ADDR_RESULT   = 0x00,
    REG_ADDR_ALERT    = 0x01,
    REG_ADDR_CONFIG   = 0x02,
    REG_ADDR_LIMITL   = 0x03,
    REG_ADDR_LIMITH   = 0x04,
    REG_ADDR_HYST     = 0x05,
    REG_ADDR_CONVL    = 0x06,
    REG_ADDR_CONVH    = 0x07,

    ADDR_ADC121       = 0x54,
};

static unsigned int adcval;    

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    
    val = val < in_min ? in_min : val;
    val = val > in_max ? in_max : val;
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sensors_init( ) {

    Wire.begin();
    Wire.beginTransmission( ADDR_ADC121 );        // transmit to device
    Wire.write( REG_ADDR_CONFIG );                // Configuration Register
    Wire.write( 0x02 );  //0x20  //0x10 works
    Wire.endTransmission(); 
    
    /* Schedule next transmission */
    os_setCallback( &sensjob, sensors_update );
}

void sensors_update( osjob_t* j ) {

    Wire.beginTransmission( ADDR_ADC121 );        // transmit to device
    Wire.write( REG_ADDR_RESULT );                // get reuslt
    Wire.endTransmission();

    Wire.requestFrom( ADDR_ADC121, 2 );           // request 2byte from device
    if( Wire.available() == 2 ) {
      adcval = (Wire.read()&0x0f)<<8;
      adcval |= Wire.read();
    }
    
    float soilm = mapf( adcval, 1580, 3690, 100.0, 0.0);

    SENSORS_PRINT("Soil Moisture: "); 
    SENSORS_PRINT(soilm);
    SENSORS_PRINT(" %\t");
    SENSORS_PRINT("   -- ADC VAL: "); 
    SENSORS_PRINTLN(adcval);
    

    /* Update Data Frame to Send */
    lpp.reset();
    lpp.addRelativeHumidity(1, soilm);
    protocol_updateDataFrame( lpp.getBuffer(), lpp.getSize() );
    os_setTimedCallback( &sensjob, os_getTime() + sec2osticks(SENSOR_INTERVAL), sensors_update );
}

#endif