/*
    Project: <https://github.com/AngelJMC/agriot-sensor-node>   
    Copyright (c) 2020 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <lmic.h>

#ifdef __cplusplus
extern "C"{
#endif

#define SENSORS_DEBUG 

/* Setup debug printing macros. */
#ifdef SENSORS_DEBUG
#define SENSORS_PRINT(...)                                                       \
  { Serial.print(__VA_ARGS__); }
#define SENSORS_PRINTLN(...)                                                     \
  { Serial.println(__VA_ARGS__); }
#define SENSORS_PRINT_F(...)                                                     \
  { Serial.print(F(__VA_ARGS__)); }
#else
#define SENSORS_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define SENSORS_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#define SENSORS_PRINT_F(...)                                                     \
  {} 
#endif


void sensors_init( );

#ifdef __cplusplus
}
#endif

#endif // _SENSORS_H_