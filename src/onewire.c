
// (C) copyright 2003 j.d.sandoz / jds-pic !at! losdos.dyndns.org

// released under the GNU GENERAL PUBLIC LICENSE (GPL)
// refer to http://www.gnu.org/licenses/gpl.txt

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

/***********************1Wire Class***********************/
/*Description: This class handles all communication */
/* between the processor and the 1wire */
/* sensors.
/*********************************************************/

#include <p18cxxx.h>

/*-------1-wire definitions-------*/
#include "onewire.h"
#include "mydelays.h"
#include <delays.h>
/*******************1-wire communication functions********************/


/************onewire_reset*************************************************/
/*This function initiates the 1wire bus */
/* */
/*PARAMETERS: */
/*RETURNS: */
/*********************************************************************/

char onewire_reset()  // OK if just using a single permanently connected device
{
    char presence;
// output_low(ONE_WIRE_PIN);
ONE_WIRE_LAT=0; // we do this once
ONE_WIRE_LOW; 
 Delay10TCYx( 49 ); // pull 1-wire low for reset pulse 500us
// output_float(ONE_WIRE_PIN); // float 1-wire high
ONE_WIRE_HIGH; 
 Delay10TCYx( 7 ); // wait-out remaining initialisation window.
 presence=ONE_WIRE_PIN;
 Delay10TCYx(38);
//output_float(ONE_WIRE_PIN);
 return !presence;
}

/*********************** onewire_write() ********************************/
/*This function writes a byte to the sensor.*/
/* */
/*Parameters: byte - the byte to be written to the 1-wire */
/*Returns: */
/*********************************************************************/

void onewire_write(char data)
{
 unsigned char count;
 unsigned char bit;
 INTCONbits.GIE=0; 
 for (count=0; count<8; ++count)
 {
  bit=data&(1<<count);   
  ONE_WIRE_LOW; // OUTPUT is already set to LOW in reset call
  Delay1TCY(); // pull 1-wire low to initiate write time-slot.
  if(bit)  ONE_WIRE_HIGH; else  ONE_WIRE_LOW;
            Delay10TCYx( 3); // wait until end of write slot.
            ONE_WIRE_HIGH;
            Delay1TCY();
         
 }
  INTCONbits.GIE=1; 
}
/*********************** read1wire() *********************************/
/*This function reads the 8 -bit data via the 1-wire sensor. */
/* */
/*Parameters: */
/*Returns: 8-bit (1-byte) data from sensor */
/*********************************************************************/

int onewire_read()
{
 int count, data;
 INTCONbits.GIE=0; 
 for (count=0; count<8; count++)
 {
//  output_low(ONE_WIRE_PIN);
  ONE_WIRE_LOW;
  Delay1TCY(); // pull 1-wire low to initiate read time-slot.
//  output_float(ONE_WIRE_PIN); // now let 1-wire float high,
  ONE_WIRE_HIGH;
  Delay10TCY( ); // let device state stabilise,
//  shift_right(&data,1,input(ONE_WIRE_PIN)); // and load result.
  data>>=1;     
  if(ONE_WIRE_PIN){data |= 0x80;}
      
  Delay10TCY( ); // wait until end of read slot.
 }
 INTCONbits.GIE=1; 

 return( data );
} 
/*********************************************************************/
