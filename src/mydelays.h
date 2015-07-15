#ifndef __DELAYS
#define __DELAYS
/*
  Name: dealay.h
  Title: Simple delay routine without interrupt to 1st programming exercises
		 Can be replaced by user with better algoritmes

  By: Rolf Ziegler
  Date: Feb 2010

  Purpose: C18 delay routines are based on CPU cycle. This code allows user friendly delays
		   the routine anticipates the cpu to run at 48mhz application speed (original product setup).

*/
#include "typedef.h"

void delay_ms(u16 del_ms);
void delay_us(u16 us);
void delay_10us(u16 us);

#define DEL2US _asm nop\ _endasm
#endif
