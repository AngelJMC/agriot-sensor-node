/*
    Project: <https://github.com/AngelJMC/agriot-sensor-node>   
    Copyright (c) 2020 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#ifdef SENS_GPSNMEA
#include "protocol.h"
#include "sensors.h"
#include <SoftwareSerial.h>
#include <MicroNMEA.h>
#include <CayenneLPP.h>


#define RX_GPS 4
#define TX_GPS 3
#define BAUD_GPS 9600
#define DEBUG_ON

//Schedule sensore measurement every this senconds
#define SENSOR_INTERVAL (5*60)//seconds

enum {
    verbose = 0,
    cayenneLLP = 1
};

SoftwareSerial gps(RX_GPS, TX_GPS);
char nmeaBuffer[128];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

static osjob_t sensjob;
static uint8_t payload[13];
CayenneLPP lpp(51);

static void sensors_update( osjob_t* j ) {
 
    nmea.clear();
    memset(nmeaBuffer, 0, sizeof(nmeaBuffer));
    
    /* Parse GPS output data stream */
    char c_gps;
    uint32_t time = millis();
    while ( millis()-time <= 1000 ) {
        if( gps.available() ){
            c_gps = gps.read();
            nmea.process(c_gps);
            
            if( verbose ) Serial.print(c_gps);
            
            if (nmea.isValid())
                break;
      }
    }

    if ( nmea.isValid() & (nmea.getNavSystem() != '\0')) {
        /* Get GPS data*/
        int32_t const lat = nmea.getLatitude(); 
        int32_t const lon = nmea.getLongitude();
        long alt = 0;
        uint16_t altitude = nmea.getAltitude(alt) ? alt/1000 : 0;
        uint8_t const hdop = nmea.getHDOP();
        uint8_t const sats = nmea.getNumSatellites();

        SENSORS_PRINT_F("\n------GPS data ------\n");
        SENSORS_PRINT_F("Nav system: ");SENSORS_PRINTLN(nmea.getNavSystem());
        SENSORS_PRINT_F("latitude: ");  SENSORS_PRINTLN(lat);
        SENSORS_PRINT_F("longitude: "); SENSORS_PRINTLN(lon);
        SENSORS_PRINT_F("altitud: ");   SENSORS_PRINTLN(altitude);
        SENSORS_PRINT_F("hdop: ");      SENSORS_PRINTLN(hdop);
        SENSORS_PRINT_F("sats: ");      SENSORS_PRINTLN(sats);
        SENSORS_PRINT_F("-----------------\n");

        /*Send with cayenneLPP format*/
        if( cayenneLLP ) {
            lpp.reset();
            float latitude  = (float)lat/1000000; 
            float longitude = (float)lon/1000000;
            long alt = 0;
            float altitude = nmea.getAltitude(alt) ? (float)alt/1000 : 0;
            lpp.addGPS(1, latitude, longitude, altitude );
             /* Update Data Frame to Send */
            protocol_updateDataFrame( lpp.getBuffer(), lpp.getSize() );
        }
        else {
            payload[0] = (byte) ((lat & 0xFF000000) >> 24 );
            payload[1] = (byte) ((lat & 0x00FF0000) >> 16 );
            payload[2] = (byte) ((lat & 0x0000FF00) >> 8 );
            payload[3] = (byte) ((lat & 0X000000FF));
            payload[4] = (byte) ((lon & 0xFF000000) >> 24 );
            payload[5] = (byte) ((lon & 0x00FF0000) >> 16 );
            payload[6] = (byte) ((lon & 0x0000FF00) >> 8 );
            payload[7] = (byte) ((lon & 0X000000FF));
            payload[8] = (byte) ((altitude & 0xFF00) >> 8);
            payload[9] = (byte) (altitude & 0x00FF);
            payload[10] = (byte) hdop; 
            payload[11] = (byte) sats; 
            
            /* Update Data Frame to Send */
            protocol_updateDataFrame( payload, sizeof(payload) - 1 );
        }
    }
    else {
        SENSORS_PRINT_F("NMEA not valid\n");
    }

    /* Schedule next sensor reading */
    os_setTimedCallback(&sensjob, os_getTime() + sec2osticks(SENSOR_INTERVAL), sensors_update);
    Serial.flush();
}

void sensors_init( ) {

    gps.begin(BAUD_GPS);    
    /*Avoid sleep on GPS mode*/
    os_avoidSleep();    
    /* Schedule the first sensor reading*/
    os_setTimedCallback(&sensjob, os_getTime() + sec2osticks(10), sensors_update);
}

#endif