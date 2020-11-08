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
#else
#define SENSORS_PRINT(...)                                                       \
  {} /**< Debug Print Placeholder if Debug is disabled */
#define SENSORS_PRINTLN(...)                                                     \
  {} /**< Debug Print Line Placeholder if Debug is disabled */
#endif


void sensors_init( );

void sensors_update(  osjob_t* j );

#ifdef __cplusplus
}
#endif

#endif // _SENSORS_H_