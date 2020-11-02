#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "Arduino.h"


#ifdef __cplusplus
extern "C"{
#endif
/* Uncomment to enable printing out nice debug messages. */
#define PROTOCOL_DEBUG

#define DEBUG_PRINTER                                                          \
  Serial /**< Define where debug output will be printed.                       \
          */

/* Setup debug printing macros. */
#ifdef PROTOCOL_DEBUG
#define PROTOCOL_PRINT(...)                                                       \
  { DEBUG_PRINTER.print(__VA_ARGS__); }
#define PROTOCOL_PRINTLN(...)                                                     \
  { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define PROTOCOL_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define PROTOCOL_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
#define TX_INTERVAL ( 5*60 )//seconds


void protocol_init( void );

void protocol_updateDataFrame( uint8_t* buff, uint8_t size );


#ifdef __cplusplus
}
#endif
#endif //_PROTOCOL_H_





