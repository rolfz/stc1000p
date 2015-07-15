
  By: Rolf Ziegler
  Date: Feb 2010

  Purpose: C18 delay routines are based on CPU cycle. This code allows user friendly delays
		   the routine anticipates the cpu to run at 48mhz application speed (original product setup).

*/

#include "typedefs.h"
#include <p18cxxx.h>

void delay_ms(u16 del_ms)
{
    long i;

#ifndef __DEBUG
    while (del_ms--)
     for (i=0; i < 150   ; i++); // was 330
#endif
}

void delay_us(char us)
{
#ifndef __DEBUG
while(us--)
	{
	_asm
	nop
	_endasm
	}
#endif
}
void delay_10us(char us)
{
#ifndef __DEBUG
    while(us--)
    {
	_asm
	nop
    nop
    nop
    nop
	_endasm
    }
#endif
}
