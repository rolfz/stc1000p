/*
 *  Header file for DS1820 temperature sensor
 * 
 *  Requires onewire.c and onewire.h files.
 * 
*/
#ifndef __DS1820__
#define __DS1820__

char ds1820_exist(void);
int ds1820_read_raw(void);

#endif
