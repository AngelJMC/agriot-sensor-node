#include <lmic.h>
#include <hal/hal.h>
#include "cfg.h"
#include "protocol.h"

enum {
    SWITCH_SAMPLE_TIME = 1,
    SWITCH_WAIT_TIME   = 10, //s
    
    GPIO_LED           = A0,        
    GPIO_SWITCH        = A1
};

struct txdata{
  uint8_t *buff;
  uint8_t size;
};


//Maping for AGRIOT v1 based on Arduino pro MINI
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 7, LMIC_UNUSED_PIN},
};

static osjob_t sendjob;
static osjob_t switchjob;
struct txdata txdata;

// These callbacks are only used in over-the-air activation
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
void os_getArtEui (u1_t* buf) {
    for( int i = 0; i < APPEUI_SIZE; ++i)
        buf[i] = cfg.appEUI[(APPEUI_SIZE-1)-i];
}

// This should also be in little endian format, see above.
void os_getDevEui (u1_t* buf) {
    for( int i = 0; i < DEVEUI_SIZE; ++i) 
        buf[i] = cfg.devEUI[(DEVEUI_SIZE-1)-i];
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
void os_getDevKey (u1_t* buf) {  
    memcpy(buf, cfg.appkey, APPKEY_SIZE);
}

void os_send( osjob_t* j ) {
    // Check if there is not a current TX/RX job running
    digitalWrite( GPIO_LED, false );
    if (LMIC.opmode & OP_TXRXPEND) {
        PROTOCOL_PRINTLN(F("OP_TXRXPEND, not sending"));
    } 
    else {
        // Prepare data transmission at the next possible time.
        LMIC_setTxData2(1, txdata.buff, txdata.size, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void os_checkSwitch( osjob_t* j ) {
    if( !digitalRead( GPIO_SWITCH ) ) {
        os_setCallback( &sendjob, os_send);
        os_setTimedCallback( &switchjob, os_getTime() + sec2osticks(SWITCH_WAIT_TIME), os_checkSwitch);
    }
    else
        os_setTimedCallback( &switchjob, os_getTime() + sec2osticks(SWITCH_SAMPLE_TIME), os_checkSwitch);
    
}
  
void protocol_init( ) {
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Cause the RX windows to open earlier to accomodate issues caused by the 
    // Arduino Mini's relatively slow (8 MHz) clock
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

    pinMode( GPIO_SWITCH, INPUT ); 
    pinMode( GPIO_LED, OUTPUT );

    // Start job (sending automatically starts OTAA too)
    os_send( &sendjob );
    os_checkSwitch( &switchjob ); 
}

void protocol_updateDataFrame( uint8_t* buff, uint8_t size ) {
    txdata.buff = buff;
    txdata.size = size;
}

void onEvent (ev_t ev) {
    PROTOCOL_PRINT(os_getTime());
    PROTOCOL_PRINT(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            PROTOCOL_PRINTLN(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            PROTOCOL_PRINTLN(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            PROTOCOL_PRINTLN(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            PROTOCOL_PRINTLN(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            PROTOCOL_PRINTLN(F("EV_JOINING"));
            break;
        case EV_JOINED:
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            PROTOCOL_PRINTLN(F("EV_JOINED"));
            break;
        case EV_JOIN_FAILED:
            PROTOCOL_PRINTLN(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            PROTOCOL_PRINTLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            PROTOCOL_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            digitalWrite( GPIO_LED, true );
            if (LMIC.txrxFlags & TXRX_ACK)
              PROTOCOL_PRINTLN(F("Received ack"));
            if (LMIC.dataLen) {
              PROTOCOL_PRINTLN(F("Received "));
              PROTOCOL_PRINTLN(LMIC.dataLen);
              PROTOCOL_PRINTLN(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback( &sendjob, os_getTime() + sec2osticks(TX_INTERVAL), os_send);
            break;
        case EV_LOST_TSYNC:
            PROTOCOL_PRINTLN(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            PROTOCOL_PRINTLN(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            PROTOCOL_PRINTLN(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            PROTOCOL_PRINTLN(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            PROTOCOL_PRINTLN(F("EV_LINK_ALIVE"));
            break;
         default:
            PROTOCOL_PRINT(F("Unknown event : "));
            PROTOCOL_PRINTLN(ev);
            break;
    }
}
