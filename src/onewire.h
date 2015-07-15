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

/*-------1-wire definitions-------*/

#ifndef __ONEWIRE__
#define __ONEWIRE__


// timing
#define DELAY500 400 // min 480 mesured 
#define DELAY120 100
#define DELAY2 2
#define DELAY6 5
#define DELAY8 6
#define DELAY60 50

#define ONE_WIRE_LAT  LATAbits.LATA0
#define ONE_WIRE_PIN  PORTAbits.RA1
#define ONE_WIRE_TRIS TRISAbits.TRISA1


#define ONE_WIRE_HIGH ONE_WIRE_TRIS=1
#define ONE_WIRE_LOW  ONE_WIRE_TRIS=0
#define ONE_WIRE_IN   ONE_WIRE_TRIS=1


char onewire_reset(void);
void onewire_write(char data);
int onewire_read(void);


 #define testbit(var, bit)       ((var) & (1 << (bit)))
 #define setbit(var, bit)        ((var) |= (1 << (bit)))
 #define clearbit(var, bit)      ((var) &= ~(1 << (bit)))

#endif
