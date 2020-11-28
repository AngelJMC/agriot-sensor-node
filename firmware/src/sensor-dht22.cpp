#ifdef SENS_DHT22
#include "protocol.h"
#include "DHT_U.h"
#include "sensors.h"
#include <CayenneLPP.h>

//Schedule sensore measurement every this senconds
#define SENSOR_INTERVAL (2*60)//seconds

static osjob_t sensjob;
CayenneLPP lpp(51);
DHT dht( 3, DHT22 ); 

static void sensors_update( osjob_t* j ) {
    os_avoidSleep();
    float t = dht.readTemperature( ); // Read temperature as Celsius
    float h = dht.readHumidity( );
    SENSORS_PRINT_F("Temperature: "); 
    SENSORS_PRINT(t);
    SENSORS_PRINT_F(" *C ");
    SENSORS_PRINT_F("Humidity: "); 
    SENSORS_PRINT(h);
    SENSORS_PRINT_F(" %\t\n");
    lpp.reset();
    lpp.addTemperature(1, t);
    lpp.addRelativeHumidity(2, h);
    protocol_updateDataFrame( lpp.getBuffer(), lpp.getSize() );
    // Schedule next sensor reading
    os_setTimedCallback( &sensjob, os_getTime() + sec2osticks(SENSOR_INTERVAL), sensors_update );
    Serial.flush();
    os_acceptSleep();
}

void sensors_init( ) {

    dht.begin();
    // Schedule the first sensor reading
    os_setCallback( &sensjob, sensors_update );
}

#endif