#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <lmic.h>

#define SAMPLING_TIME 30//seconds

#define SENSORS_DEBUG 

#define DEBUG_PRINTER Serial /**< Define where debug output will be printed.*/

/* Setup debug printing macros. */
#ifdef SENSORS_DEBUG
#define SENSORS_PRINT(...)                                                       \
  { DEBUG_PRINTER.print(__VA_ARGS__); }
#define SENSORS_PRINTLN(...)                                                     \
  { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define SENSORS_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define SENSORS_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif

void sensors_init( );


void sensors_update(  osjob_t* j );



#endif // _SENSORS_H_