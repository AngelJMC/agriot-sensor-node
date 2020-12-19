/*
    Project: <https://github.com/AngelJMC/agriot-sensor-node>   
    Copyright (c) 2020 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#ifndef _CFG_H_
#define _CFG_H_

#include "Arduino.h"

#ifdef __cplusplus
extern "C"{
#endif

enum {
    DEVEUI_SIZE = 8,
    APPEUI_SIZE = 8,
    APPKEY_SIZE = 16,
    
    NWKSKEY_SIZE = 16,
    APPSKEY_SIZE = 16
};

struct cfg{
    uint8_t devEUI[DEVEUI_SIZE];    /* Device EUI */
    uint8_t appEUI[APPEUI_SIZE];    /* Application EUI */
    uint8_t appkey[APPKEY_SIZE];    /* Application key */
    
    
    uint8_t nwkskey[NWKSKEY_SIZE];  /* LoraWAN Network Session Key*/
    uint8_t appskey[APPSKEY_SIZE];  /* LoraWAN Application Session Key*/
    uint32_t devaddr;  /* LoraWAN end node device address */
    uint32_t seqnoUp;
    uint8_t otaajoin;
}; 

extern struct cfg cfg;

void param_load( void ); 

void param_savecfg( );

#ifdef __cplusplus
}
#endif

#endif //_CFG_H_