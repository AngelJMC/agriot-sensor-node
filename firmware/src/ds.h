

struct dssens {
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
};



void ds_init( struct dssens* ds_hdl );

float ds_readTemperature( struct dssens* ds_hdl );
