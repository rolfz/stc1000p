/*
 * 
 *  Driver for Temperature sensor DS18x20, based on CCS driver
 *  
 * Code trimmed to run with processor running at 1mhz, 1us instruction time.
 * 
 * By Rolf Ziegler, July 2015
 * 
 */
#include "onewire.h"
#include <p18cxxx.h>

/* Routine to check if a sensor exists on the 1wire bus */
char ds1820_exist()
{
    return onewire_reset();
}

/* Routine reads 1Wire pulses, IO defined in Onewire.h file.
 * 
 * Returns DS18x20 raw format, means 12bit temperature + 4 bits *.0625 deg.
 */
int ds1820_read_raw(void)
{
 unsigned char busy=0, data[2];
 int raw;

// INTCONbits.GIE=0;  

 onewire_reset();
 onewire_write(0xCC);
 onewire_write(0x44);

 while (busy == 0)
  busy = onewire_read();

 
 onewire_reset();
 onewire_write(0xCC);
 onewire_write(0xBE); // read scratchpad

 data[0] = onewire_read();
 data[1] = onewire_read();
 raw=((int)data[1]*256) |data[0];  
 
 INTCONbits.GIE=1;

 return raw;
} 

