#ifdef SENS_DHT22
#include "protocol.h"
#include "DHT_U.h"
#include "sensors.h"
#include <CayenneLPP.h>

static osjob_t sensjob;
CayenneLPP lpp(51);

DHT dht( 3, DHT22 ); 


void sensors_init( ) {

    dht.begin();
    // Schedule next transmission
    os_setCallback( &sensjob, sensors_update );
}


void sensors_update( osjob_t* j ) {

    float t = dht.readTemperature( ); // Read temperature as Celsius
    float h = dht.readHumidity( );
    SENSORS_PRINT("Temperature: "); 
    SENSORS_PRINT(t);
    SENSORS_PRINT(" *C ");
    SENSORS_PRINT("Humidity: "); 
    SENSORS_PRINT(h);
    SENSORS_PRINTLN(" %\t");
    lpp.reset();
    lpp.addTemperature(1, t);
    lpp.addRelativeHumidity(2, h);
    protocol_updateDataFrame( lpp.getBuffer(), lpp.getSize() );

    os_setTimedCallback( &sensjob, os_getTime() + sec2osticks(SAMPLING_TIME), sensors_update );
}

#endif