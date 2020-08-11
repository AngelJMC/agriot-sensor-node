#include "protocol.h"
#include "DHT_U.h"
#include "ds.h"
#include "sensors.h"
#include <CayenneLPP.h>


CayenneLPP lpp(51);


#ifdef SENS_DS
static dssens ds_hdl;
#elif SENS_DHT22
enum {
    DHTPIN = 3,     // what pin we're connected to
    DHTTYPE = DHT22   // DHT 22  (AM2302)
};
DHT dht( DHTPIN, DHTTYPE ); 
#endif


static osjob_t sensjob;




void sensors_init( ) {

#ifdef SENS_DS
      ds_init( &ds_hdl );
#elif SENS_DHT22
      dht.begin();
#endif

// Schedule next transmission
os_setCallback( &sensjob, sensors_update );

}


void sensors_update( osjob_t* j ) {
    struct txdata* data = &txdata;
    #if SENS_DS

        static float t = ds_readTemperature( &ds_hdl );
        SENSORS_PRINT("Temperature: "); 
        SENSORS_PRINT(t);
        SENSORS_PRINTLN(" *C ");
        lpp.reset();
        lpp.addTemperature(1, t);
        data->buff = lpp.getBuffer();
        data->len  = lpp.getSize();

    #elif SENS_DHT22

        static float t = dht.readTemperature( ); // Read temperature as Celsius
        static float h = dht.readHumidity( );
        SENSORS_PRINT("Temperature: "); 
        SENSORS_PRINT(t);
        SENSORS_PRINT(" *C ");
        SENSORS_PRINT("Humidity: "); 
        SENSORS_PRINT(h);
        SENSORS_PRINTLN(" %\t");
        lpp.reset();
        lpp.addTemperature(1, t);
        lpp.addRelativeHumidity(2, h);
        data->buff = lpp.getBuffer();
        data->len  = lpp.getSize();

    #endif

    os_setTimedCallback( &sensjob, os_getTime() + sec2osticks(SAMPLING_TIME), sensors_update );

}