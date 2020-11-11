#ifdef SENS_5TM
#include "protocol.h"
#include "SDISerial.h"
#include "sensors.h"
#include <CayenneLPP.h>

//Schedule sensore measurement every this senconds
#define SENSOR_INTERVAL (1*30)//seconds

enum {
    INVERTED = 1,
    DATALINE_PIN = 3
};

static osjob_t sensjob;
CayenneLPP lpp(51);
SDISerial sdi12(DATALINE_PIN, INVERTED);

static void sensors_update( osjob_t* j ) {


    sdi12.sdi_query("?M!",1000);
    sdi12.wait_for_response(1000);
    char* response = sdi12.sdi_query("?D0!",1000);
    SENSORS_PRINT_F("RECV:");
    SENSORS_PRINTLN(response!=NULL&&response[0] != '\0'?response:"No Response!"); //just a debug print statement to the serial port

    char *token;
    const char s[] = "+";
    /* get the first token */
    token = strtok(response, s);
    float data[2];
    /* walk through other tokens */
    uint8_t i = 0;
    token = strtok(NULL, s);
    
    while( token != NULL ) { 
       data[i++] = atof(token);
       Serial.println( data[i-1]);
       token = strtok(NULL, s);
    }

    float soilMois = 0.0000043*pow(data[0],3) - 0.00055*pow(data[0],2) + 0.0292*data[0]-0.053;

    SENSORS_PRINT_F("Temperature: "); 
    SENSORS_PRINT(data[1]);
    SENSORS_PRINT_F(" *C\n");
    SENSORS_PRINT_F("Soil Moisture: "); 
    SENSORS_PRINT(soilMois);
    SENSORS_PRINT_F(" *H\n");
    lpp.reset();
    lpp.addTemperature(1, data[1]);
    lpp.addGenericSensor(2,soilMois);
    protocol_updateDataFrame( lpp.getBuffer(), lpp.getSize() );

    os_setTimedCallback( &sensjob, os_getTime() + sec2osticks(SENSOR_INTERVAL), sensors_update );
}

void sensors_init( ) {

    sdi12.begin(); // start our SDI connection 
    delay(1000);//3 seconds should be more than enough
    char* sensor_info = sdi12.sdi_query("?I!",1000); // get sensor info
    SENSORS_PRINT_F("Sensor Info:");
    SENSORS_PRINTLN( sensor_info?sensor_info:"No Response");
    
    // Schedule next transmission
    os_setCallback( &sensjob, sensors_update );
}

#endif