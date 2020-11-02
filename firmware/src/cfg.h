
#ifndef _CFG_H_
#define _CFG_H_

#include "Arduino.h"

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

#endif //_CFG_H_