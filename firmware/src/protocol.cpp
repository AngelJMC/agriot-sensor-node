#include <lmic.h>
#include <hal/hal.h>
#include "cfg.h"
#include "protocol.h"

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
#define TX_INTERVAL ( 5*60 )//seconds
#define ACK_DOWNLINK_INTERVAL ( (int32_t)24*60*60 ) / ( TX_INTERVAL*10 )

enum {    
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
struct txdata txdata;
static uint8_t sendAck;
static uint8_t lastStatuSwitch;

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
        os_avoidSleep();
        if ( sendAck >= ACK_DOWNLINK_INTERVAL )
            sendAck = 0;
        
        LMIC_setTxData2(1, txdata.buff, txdata.size, sendAck == 0);
        Serial.print(F("Packet queued, packets to next ack: "));
        Serial.println(ACK_DOWNLINK_INTERVAL - sendAck);
        ++sendAck;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void protocol_checkSwitch(  ) {
    uint8_t statusSwitch = digitalRead( GPIO_SWITCH );
    if( !statusSwitch && lastStatuSwitch ) {
        os_setCallback( &sendjob, os_send);
    }
    lastStatuSwitch = statusSwitch;
    }
  
void protocol_init( ) {
    // Reset the MAC state. Session and pending data transfers will be discarded.
    os_avoidSleep();
    LMIC_reset();
    // Cause the RX windows to open earlier to accomodate issues caused by the 
    // Arduino Mini's relatively slow (8 MHz) clock
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    pinMode( GPIO_SWITCH, INPUT ); 
    pinMode( GPIO_LED, OUTPUT );

    os_send( &sendjob );
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
            /* Set default/start data rate  and transmit power for uplink*/
            LMIC_setDrTxpow(DR_SF9,14);
            /* set ADR mode (if mobile turn off) */
            LMIC_setAdrMode(1);
            PROTOCOL_PRINTLN(F("EV_JOINING"));
            break;
        case EV_JOINED:
            LMIC_setLinkCheckMode(0);
            sendAck = 0;
            os_acceptSleep();
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
            if (LMIC.txrxFlags & TXRX_ACK){
                PROTOCOL_PRINTLN(F("Received ack"));
            }
            if (LMIC.dataLen) {
                PROTOCOL_PRINT(F("Received "));
                PROTOCOL_PRINT(LMIC.dataLen);
                PROTOCOL_PRINTLN(F(" bytes of payload"));
            }
            LMIC_setLinkCheckMode(0);
            // Schedule next transmission
            os_setTimedCallback( &sendjob, os_getTime() + sec2osticks(TX_INTERVAL), os_send);
            Serial.flush();
            os_acceptSleep();
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
