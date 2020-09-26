
#ifndef _CFG_H_
#define _CFG_H_

#include "Arduino.h"

struct cfg{
    uint8_t devEUI[8];
    uint8_t appEUI[8];
    uint8_t appkey[16];
}; 


extern struct cfg cfg;

#endif //_CFG_H_