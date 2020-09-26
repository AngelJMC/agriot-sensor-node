#include <lmic.h>
#include <hal/hal.h>
#include "cfg.h"

#include "protocol.h"



enum {
    SWITCH_WAIT_TIME = 5000 //ms
};

struct txdata{
  uint8_t *buff;
  uint8_t size;
};

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
void os_getArtEui (u1_t* buf) {
    for( int i = 0; i < 8; ++i)
        buf[i] = cfg.appEUI[7-i];
}

// This should also be in little endian format, see above.
void os_getDevEui (u1_t* buf) {
    for( int i = 0; i < 8; ++i) 
        buf[i] = cfg.devEUI[7-i];
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x04, 0x14, 0xE7, 0x35, 0x8A, 0x04, 0x29, 0x1A, 0x85, 0x13, 0xE7, 0xC2, 0x1F, 0x2F, 0x35, 0x2F };
void os_getDevKey (u1_t* buf) {  
    memcpy(buf, cfg.appkey, 16);
}


void do_send(osjob_t* j);

static osjob_t sendjob;

static uint64_t lastmillis = 0;

struct txdata txdata;

//Maping for IOTMCU arduino pro MINI
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 7, LMIC_UNUSED_PIN},
};


void protocol_doSend( void ) {
    if( (millis() - lastmillis) > SWITCH_WAIT_TIME ) {
        os_setCallback( &sendjob, do_send);
        lastmillis = millis();
    }
}

void protocol_updateDataFrame( uint8_t* buff, uint8_t size ) {
    txdata.buff = buff;
    txdata.size  = size;
}
    

void do_send( osjob_t* j ) {
    digitalWrite( A0, false );
    if (LMIC.opmode & OP_TXRXPEND) {
        PROTOCOL_PRINTLN(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare data transmission at the next possible time.
        LMIC_setTxData2(1, txdata.buff, txdata.size, 0);
        PROTOCOL_PRINTLN(F("SENDING...."));
    }
}


void protocol_init( ) {

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    // Cause the RX windows to open earlier to accomodate issues caused by the 
    // Arduino Mini's relatively slow (8 MHz) clock
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);
    
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
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
        case EV_RFU1:
            PROTOCOL_PRINTLN(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            PROTOCOL_PRINTLN(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            PROTOCOL_PRINTLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            PROTOCOL_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            digitalWrite( A0, true );
            if (LMIC.txrxFlags & TXRX_ACK)
              PROTOCOL_PRINTLN(F("Received ack"));
            if (LMIC.dataLen) {
              PROTOCOL_PRINTLN(F("Received "));
              PROTOCOL_PRINTLN(LMIC.dataLen);
              PROTOCOL_PRINTLN(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback( &sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            PROTOCOL_PRINTLN(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            PROTOCOL_PRINTLN(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            PROTOCOL_PRINTLN(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            PROTOCOL_PRINTLN(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            PROTOCOL_PRINTLN(F("EV_LINK_ALIVE"));
            break;
         default:
            PROTOCOL_PRINTLN(F("Unknown event"));
            break;
    }
}
