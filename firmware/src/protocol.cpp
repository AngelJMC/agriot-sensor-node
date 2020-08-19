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
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
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


void protocol_init( struct cfg* cfg ) {

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    LMIC_setSession (0x1, cfg->devaddr, cfg->nwkskey, cfg->appskey );


    #if defined( CFG_eu868 )
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined( CFG_us915 )
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode( 0 );

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow( DR_SF7, 14 );

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
