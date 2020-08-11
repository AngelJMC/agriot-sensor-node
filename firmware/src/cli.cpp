#include "Arduino.h"
#include "cfg.h"

#include "AsyncSerialLib.h"
#include <EEPROM.h>

#define EOL            '\n'

enum loglevel{
    ECHO = 0,
    DISABLE = 0,
    DEBUG,
    WARNING,
    ERROR
};

int const dataLength = 40;
byte data[dataLength];

enum resp{
  NOK   = -1,
  NOTHING = 0,
  OK      = 1
};

typedef struct ack{
  enum resp rp;
  char type;
}ack_t;

static int const eeadrInfo = 0; 

char cmd[40];
char txbuff[64];

enum loglevel verbose = DISABLE;

struct cfg cfg;


int incomingByte = 0;

void sendresponse( ack_t *ack);
ack_t parseCommands(AsyncSerial &serial );
void printdebug( const char * msg, int errlevel );

AsyncSerial asyncSerial(data, dataLength,
	[](AsyncSerial& sender) { 
        ack_t ack = parseCommands( sender ); 
        sendresponse( &ack ); 
    }
);


void cli_update( ) {
  asyncSerial.AsyncRecieve();
  }

int getNum(char ch)
{
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

uint8_t hex2int( char hex[])
{
    return getNum(hex[0])*16 + getNum(hex[1]);
}

void printdebug( const char * msg, int errlevel ) {

    char header[4];
    sprintf( header, "!%d,", errlevel );
    Serial.print( header );
    Serial.print( msg );
    Serial.print( EOL );
}



ack_t parseCommands(AsyncSerial &serial ) {

  ack_t ack = { .rp = NOK, .type = '\0' };
  memcpy(&cmd, serial.GetContent(), serial.GetContentLength());

  if ( ECHO )  //VERBOSE ECHO
    printdebug( cmd, DEBUG );
  switch ( cmd[0] ) {
    case 'N':
      {
          if( strlen(cmd) != 34 ) 
              return ack;

          for( int i = 2; i < 34; i = i +2) {
                  char cbyte[3]="";
                  cbyte[0] = cmd[i];
                  cbyte[1] = cmd[i+1];
                  cfg.nwkskey[i/2 - 1] = hex2int(cbyte);
          }
          EEPROM.put( eeadrInfo, cfg );
      }
      break;
    case 'A':
      {
          if( strlen(cmd) != 34 ) 
              return ack;

          for( int i = 2; i < 34; i = i +2) {
                  char cbyte[3]="";
                  cbyte[0] = cmd[i];
                  cbyte[1] = cmd[i+1];
                  cfg.appskey[i/2 - 1] = hex2int(cbyte);
          }
          EEPROM.put( eeadrInfo, cfg );
      }
      break;
    case 'D':
          {
              if( strlen(cmd) != 10 ) 
                  return ack;

              uint32_t x = 0;
              for( int i = 2; i < 10; ++i) 
                    x = x*16 + getNum( cmd[i] );
          
              cfg.devaddr = x;
              EEPROM.put( eeadrInfo, cfg );
          }
      break;
    case 'I':
    {
        Serial.print("NetKey: ");
        for( int i = 0; i < 16; ++i) { 
          Serial.print(cfg.nwkskey[i], HEX );
        }
        Serial.print("\nAppKey: ");
        for( int i = 0; i < 16; ++i) { 
          Serial.print(cfg.appskey[i], HEX );
        }
      Serial.print(F("\nSensor Addr: "));
      Serial.println( cfg.devaddr, HEX );
      //Serial.print( EOL );
      }
      break;

    default:
      break;
  }
    

    ack.rp = OK;

  return ack;
}

void sendresponse( ack_t *ack) {

  if( ack->rp == OK ) { 
    Serial.print( ack->type );
    Serial.print("OK");
    Serial.print( EOL );
  }
  else if( ack->rp == NOK ) {
    Serial.print( ack->type );
    Serial.print("NOK");
    Serial.print( EOL );
  }
  memset( &cmd, '\0' , sizeof(cmd) );
}


#define CFG_VER 1


void cli_init( void ) {
    asyncSerial.FinishChar = NEW_LINE;
    asyncSerial.IgnoreChar = CARRIAGE_RETURN;
}

void param_setdefault( struct cfg* cfg ) { 
  memcpy_P(cfg->appskey, 0, sizeof(16));
  memcpy_P(cfg->nwkskey, 0, sizeof(16));
  cfg->devaddr = 0;
  
}

void param_load(  ) {
    int cfgversion=0;
    int eeAdress = eeadrInfo;
    EEPROM.get( eeAdress, cfg );
    eeAdress += sizeof( struct cfg );
    EEPROM.get( eeAdress, cfgversion );
    
    if ( cfgversion != CFG_VER ) {
      param_setdefault( &cfg );
      eeAdress = 0;
      EEPROM.put( eeAdress, cfg );
      eeAdress += sizeof( struct cfg );
      cfgversion = CFG_VER;
      EEPROM.put( eeAdress, cfgversion );
      Serial.print(F("LOAD DEFAULT\n"));
    }
} 
