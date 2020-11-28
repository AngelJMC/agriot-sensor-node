
#include "cfg.h"
#include "AsyncSerialLib.h"
#include <lmic.h>

#define EOL            '\n'


enum {
    CLI_ACTIVE_TIME  = 3*60, /*CLI is only active for 3 minutes after power-up.*/
    CLI_JOB_INTERVAL = 250   /*Job interval execution in milliseconds*/
};

enum loglevel{
    ECHO = 0,
    DISABLE = 0,
    DEBUG,
    WARNING,
    ERROR
};

enum resp{
  NOK   = -1,
  NOTHING = 0,
  OK      = 1
};

typedef struct ack{
  enum resp rp;
  char type;
}ack_t;

enum loglevel verbose = DISABLE;

int const dataLength = 64;
byte data[dataLength];
static osjob_t clijob;


void sendresponse( ack_t *ack);
ack_t parseCommands(AsyncSerial &serial );
void printdebug( const char * msg, int errlevel );

AsyncSerial asyncSerial( data, dataLength, [](AsyncSerial& sender) { 
        ack_t ack = parseCommands( sender ); 
        sendresponse( &ack ); 
    }
);

int getNum( char ch ) {
    int num=0;
    if( ch>='0' && ch<='9' ) {
        num=ch-0x30;
    }
    else {
        switch( ch ) {
            case 'A': case 'a': num=10; break;
            case 'B': case 'b': num=11; break;
            case 'C': case 'c': num=12; break;
            case 'D': case 'd': num=13; break;
            case 'E': case 'e': num=14; break;
            case 'F': case 'f': num=15; break;
            default: num=0;
        }
    }
    return num;
}

uint8_t hex2int( char hex[] ) {
    return getNum(hex[0])*16 + getNum(hex[1]);
}

int parseStringHex( uint8_t* dest, const char* cmd, size_t len ) {
    if( strlen(cmd) != len ) 
        return -1;

    for( size_t i = 2; i < len; i = i +2) {
        char cbyte[3]="";
        cbyte[0] = cmd[i];
        cbyte[1] = cmd[i+1];
        dest[i/2 - 1] = hex2int(cbyte);
    }
    return 0;
}

void printStringHex( const uint8_t* source, size_t len ) {
    for( size_t i = 0; i < len; ++i) {
        if (source[i] < 0x10)
			      Serial.print("0");
        Serial.print(source[i], HEX );
    }
}

void printdebug( const char* msg, int errlevel ) {

    char header[4];
    sprintf( header, "!%d,", errlevel );
    Serial.print( header );
    Serial.print( msg );
    Serial.print( EOL );
}



ack_t parseCommands(AsyncSerial &serial ) {

    ack_t ack = { .rp = NOK, .type = '\0' };
    const char* cmd = (char*)serial.GetContent();
    if ( ECHO )  //VERBOSE ECHO
        printdebug( cmd, DEBUG );
    
    switch ( cmd[0] ) {
        case 'E': {
            if( cmd[1] == ':') {
                int err = parseStringHex( cfg.appEUI, cmd, (APPEUI_SIZE*2)+2 );
                if( err )
                    return ack;
                param_savecfg( );
            }
            
            Serial.print(F("App EUI: "));
            printStringHex( cfg.appEUI, APPEUI_SIZE);
            Serial.print( EOL );

            break;
        }
        
        case 'K': {
            if( cmd[1] == ':') {
                int err = parseStringHex( cfg.appkey, cmd, (APPKEY_SIZE*2)+2 );
                if( err )
                    return ack;
                param_savecfg( );
            }
            
            Serial.print(F("App Key: "));
            printStringHex( cfg.appkey, APPKEY_SIZE);
            Serial.print( EOL );
            

            break;
        }
        case 'D': {
            Serial.print(F("Dev EUI: "));
            printStringHex( cfg.devEUI, DEVEUI_SIZE);
            Serial.print( EOL );  
            
            break;
        }

        default:
            break;
    }
    memset ( data, 0, sizeof(byte)*dataLength);
    ack.rp = OK;
    return ack;
}

void sendresponse( ack_t *ack) {

    switch( ack->rp ) {
        case OK : Serial.print(F("OK")); break;
        case NOK: Serial.print(F("NOK")); break;
        default: break;
    }
    Serial.print( EOL );
}


void cli_update( osjob_t* j ) {
    asyncSerial.AsyncRecieve();
    if( osticks2sec(os_getTime()) < CLI_ACTIVE_TIME )
        os_setTimedCallback( &clijob, os_getTime() + ms2osticks(CLI_JOB_INTERVAL), cli_update );
}


void cli_init( void ) {
    asyncSerial.FinishChar = NEW_LINE;
    asyncSerial.IgnoreChar = CARRIAGE_RETURN;
    os_setTimedCallback( &clijob, os_getTime() +ms2osticks(CLI_JOB_INTERVAL), cli_update );
}



