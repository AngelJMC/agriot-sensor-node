#include "cfg.h" 
#include "ArduinoUniqueID.h"
#include <EEPROM.h>

#define CFG_VER 1

struct cfg cfg;
static int const eeadrInfo = 0; 

void param_setdefault( struct cfg* cfg ) { 
    memset(cfg->appkey, 0, APPKEY_SIZE);
    memset(cfg->appEUI, 0, APPEUI_SIZE);  
}

void param_savecfg( ) {
    EEPROM.put( eeadrInfo, cfg );
}

void param_load(  ) {
    int cfgversion=0;
    int eeAdress = eeadrInfo;
    EEPROM.get( eeAdress, cfg );
    eeAdress += sizeof( struct cfg );
    EEPROM.get( eeAdress, cfgversion );

	for (size_t i = 0; i < 8; i++)
        cfg.devEUI[i]=UniqueID8[i];
    
    if ( cfgversion != CFG_VER ) {
        param_setdefault( &cfg );
        eeAdress = 0;
        EEPROM.put( eeAdress, cfg );
        eeAdress += sizeof( struct cfg );
        cfgversion = CFG_VER;
        EEPROM.put( eeAdress, cfgversion );
        Serial.print(F("LOAD DEFAULT\n"));
    }
} 