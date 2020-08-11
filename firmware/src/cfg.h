
#ifndef _CFG_H_
#define _CFG_H_

#include "Arduino.h"

struct cfg{
    uint8_t nwkskey[16];
    uint8_t appskey[16];
    uint32_t devaddr;
}; 


extern struct cfg cfg;

#endif //_CFG_H_