 


#ifdef __cplusplus
extern "C"{
#endif

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = {  };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = {  };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000; // <-- Change this address for every node!

#ifdef __cplusplus
}
#endif
