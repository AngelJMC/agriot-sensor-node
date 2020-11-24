
#ifndef _CFG_H_
#define _CFG_H_

#include "Arduino.h"

#ifdef __cplusplus
extern "C"{
#endif

enum {
    DEVEUI_SIZE = 8,
    APPEUI_SIZE = 8,
    APPKEY_SIZE = 16
};

struct cfg{
    uint8_t devEUI[DEVEUI_SIZE];
    uint8_t appEUI[APPEUI_SIZE];
    uint8_t appkey[APPKEY_SIZE];
}; 

extern struct cfg cfg;

void param_load( void ); 

void param_savecfg( );

#ifdef __cplusplus
}
#endif

#endif //_CFG_H_