/*
    Project: <https://github.com/AngelJMC/AGRIOT_lora-sensor-node>   
    Copyright (c) 2020 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "Arduino.h"

#ifdef __cplusplus
extern "C"{
#endif

/* Uncomment to enable printing out nice debug messages. */
#define PROTOCOL_DEBUG


/* Setup debug printing macros. */
#ifdef PROTOCOL_DEBUG
#define PROTOCOL_PRINT(...)                                                       \
  { Serial.print(__VA_ARGS__); }
#define PROTOCOL_PRINTLN(...)                                                     \
  { Serial.println(__VA_ARGS__); }
#else
#define PROTOCOL_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define PROTOCOL_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif


void protocol_init( void );

void protocol_updateDataFrame( uint8_t* buff, uint8_t size );

void protocol_checkSwitch(  );

#ifdef __cplusplus
}
#endif
#endif //_PROTOCOL_H_





