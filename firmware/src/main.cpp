/*******************************************************************************

 *
 *******************************************************************************/
#include "Arduino.h"
#include "cfg.h"
#include "cli.h"
#include "protocol.h"
#include "sensors.h"
#include "lmic.h"


void setup() {
  
    Serial.begin(9600);
    Serial.println(F("Starting"));

    param_load(  );
    
    os_init();
    protocol_init();
    cli_init();
    sensors_init();

}


void loop() {
    os_runloop_once( ); 
    protocol_checkSwitch( );
}
