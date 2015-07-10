/*
 * STC1000+, improved firmware and Arduino based firmware uploader for the STC-1000 dual stage thermostat.
 *
 * Copyright 2014 Mats Staffansson
 *
 * This file is part of STC1000+.
 *
 * STC1000+ is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * STC1000+ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with STC1000+.  If not, see <http://www.gnu.org/licenses/>.
 *
 * 
 * ADDED BY ROLF ZIEGLER DURING PIC18f Implementation
 * 
 * B2 means 2nd sensor
 * COM means communication enabled
 * Timer0   433mhz transmission
 * Timer1                                           32khz timer to get 1sec pulses
 * Timer2   display led data interrupt 1ms cycle    general 1ms timer
 * Timer3
 * Timer4   reading sensor time
 * Timer6   handles buttons pressed        
 * 
 */


#include <p18cxxx.h>

#include "stc1000p.h"

/* Defines */

/* Configuration words */
///unsigned int __at _CONFIG1 __CONFIG1 = 0xFD4;
///unsigned int __at _CONFIG2 __CONFIG2 = 0x3AFF;

#pragma config OSC = INTIO67	// You can write this way
#pragma config FCMEN = OFF		  // OR this way 
#pragma config MCLRE = ON
#pragma config IESO = OFF 
#pragma config PWRT = OFF // power up timer
#pragma config BOREN = OFF                                     
//#pragma config BORV = 3
//#pragma config VREGEN = OFF
#if defined (__DEBUG)
#pragma config WDT = OFF
#else
#pragma config WDT = ON
#endif
#pragma config CP0 = OFF, CP1 = OFF //, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF //, WRT2 = OFF, WRT3 = OFF
#pragma config LVP = OFF
#pragma config DEBUG = ON//#pragma config LPT1OSC = ON

/* Temperature lookup table  */
#ifdef FAHRENHEIT
const int ad_lookup[] = { 0, -555, -319, -167, -49, 48, 134, 211, 282, 348, 412, 474, 534, 593, 652, 711, 770, 831, 893, 957, 1025, 1096, 1172, 1253, 1343, 1444, 1559, 1694, 1860, 2078, 2397, 2987 };
#else  // CELSIUS
//const int ad_lookup[] = { 0, -486, -355, -270, -205, -151, -104, -61, -21, 16, 51, 85, 119, 152, 184, 217, 250, 284, 318, 354, 391, 431, 473, 519, 569, 624, 688, 763, 856, 977, 1154, 1482 };

// table starting at -25deg with value 0, increasing by 5 deg per step up to 130deg C = 32 values
// Source of the table is an interpolation based on mesured values (-10deg freezer, 0deg melting ice, 100deg boiling water)
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
const int ad_lookup[] = {-384, -358, -329, -295, -259, -220, -178, -135, -92, -45, -1, 44, 85, 127, 163, 203, 231, 263, 287, 312, 333, 353, 370, 386, 400, 412, 422, 432, 440,448,454,510 };

#endif

#define TCORRECTION 600 // offset from unknown reason, allows to adjust deg. C with PIC18F2520


/* LED character lookup table (0-15), includes hex */
//unsigned const char led_lookup[] = { 0x3, 0xb7, 0xd, 0x25, 0xb1, 0x61, 0x41, 0x37, 0x1, 0x21, 0x5, 0xc1, 0xcd, 0x85, 0x9, 0x59 };
/* LED character lookup table (0-9) */
unsigned const char led_lookup[] = { LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LED_9 };

/* Global variables to hold LED data (for multiplexing purposes) */
led_e_t led_e = {0};
led_t led_10, led_1, led_01 ={0};

//char rom digi[4]=  {0xf0-0x80,0xf0-0x20,0xf0-0x40,0xf0-0x10};
    
int temperature=0;
    
#define SAMPLES 64L

#if defined PB2
static int temperature2=0;
#elif defined COM
static volatile unsigned char com_data=0;
static volatile unsigned char com_write=0;
static volatile unsigned char com_tmout=0;
static volatile unsigned char com_count=0;
static volatile unsigned char com_state=0;
#elif defined FO433
static volatile unsigned char fo433_data=0;
static unsigned char fo433_state=0;
static volatile unsigned char fo433_count=0;
static volatile unsigned char fo433_send_space=0;
static unsigned char fo433_sec_count = 24;
#endif

#if defined(__18F2520)
// we simulate timer 4 and 6 in software
// timer 6 with 60ms interval
unsigned char TMR6IF=0;
unsigned char tmr6cnt=0;
#define TMR6_MAX  112 // 112ms count to get 1 sec pulse
// timer 4 with 120ms interval
unsigned char TMR4IF=0;
unsigned char TMR4ON=1;
unsigned char tmr4cnt=0;
#define TMR4_MAX 60 // 60ms count freq

//char TMR1GE=0;   // Menu on flag, avoid display temp when menu is on.
char menuON=0;
char alarmFlash=0; 
//RX9=0;      // SA flash flag
//extern char TX9; // 0=sens0 active, 1=sens1 active

#endif

/* Functions.
 * Note: Functions used from other page cannot be static, but functions
 * not used from other page SHOULD be static to decrease overhead.
 * Functions SHOULD be defined before used (ie. not just declared), to
 * decrease overhead. Refer to SDCC manual for more info.
 */

/* Read one configuration data from specified address.
 * arguments: Config address (0-127)
 * return: the read data
 */
unsigned int eeprom_read_config(unsigned char eeprom_address){
	unsigned int data = 0;
	eeprom_address = (eeprom_address << 1);

	do {
		EEADR = eeprom_address; // Data Memory Address to read
		EECON1bits.EEPGD = 0; // Point to DATA memory
		EECON1bits.CFGS = 0; // Deselect config space
		EECON1bits.RD = 1; // Enable read

		data = ((((unsigned int) EEDATA) << 8) | (data >> 8));
	} while(!(eeprom_address++ & 0x1));

	return data; // Return data
}

/* Store one configuration data to the specified address.
 * arguments: Config address (0-127), data
 * return: nothing
 */
void eeprom_write_config(unsigned char eeprom_address,unsigned int data)
{
	// Avoid unnecessary EEPROM writes
	if(data == eeprom_read_config(eeprom_address)){
		return;
	}

	// multiply address by 2 to get eeprom address, as we will be storing 2 bytes.
	eeprom_address = (eeprom_address << 1);

	do {
		// Address to write
	    EEADR = eeprom_address;
	    // Data to write
	    EEDATA = (unsigned char) data;
	    //Point to DATA memory
	    EECON1bits.EEPGD = 0;
	    // Deselect configuration space
	    EECON1bits.CFGS = 0;
	    // Enable write
	    EECON1bits.WREN = 1;

	    // Disable interrupts during write
	    INTCONbits.GIE=0;

	    // Write magic words to EECON2
	    EECON2 = 0x55;
	    EECON2 = 0xAA;

	    // Initiate a write cycle
	    EECON1bits.WR = 1;

	    // Re-enable interrupts
	    INTCONbits.GIE=1;

	    // Disable writes
	    EECON1bits.WREN = 0;

	    // Wait for write to complete
	    while(EECON1bits.WR);

	    // Clear write complete flag (not really needed
	    // as we use WR for check, but is nice)
	    PIR2bits.EEIF=0;

	    // Shift data for next pass
	    data = data >> 8;

	} while(!(eeprom_address++ & 0x01)); // Run twice for 16 bits

}

unsigned int divu10(unsigned int n) {
	unsigned int q, r;
	q = (n >> 1) + (n >> 2);
	q = q + (q >> 4);
	q = q + (q >> 8);
	q = q >> 3;
	r = n - ((q << 3) + (q << 1));
	return q + ((r + 6) >> 4);
}

/* Update LED globals with temperature or integer data.
 * arguments: value (actual temperature multiplied by 10 or an integer)
 *            decimal indicates if the value is multiplied by 10 (i.e. a temperature)
 * return: nothing
 */
#if defined (__18F2520)
void value_to_led(int value, unsigned char decimal) {
	unsigned char i;

	// Handle negative values
	if (value < 0) {
		led_e.e_negative = 1;
		value = -value;
	} else {
		led_e.e_negative = 0;
	}

	// This assumes that only temperatures and all temperatures are decimal
	if(decimal){
		led_e.e_deg = 1;
#ifdef FAHRENHEIT
		led_e.e_c = 1;
#else
		led_e.e_c = 0;
#endif // FAHRENHEIT
	}
    else led_e.e_deg = 0;
    
	// If temperature >= 100 we must lose decimal...
	if (value >= 1000) {
		value = divu10((unsigned int) value);
		decimal = 0;
	}

	// Convert value to BCD and set LED outputs
	if(value >= 100){
		for(i=0; value >= 100; i++){
			value -= 100;
		}
		led_10.raw = led_lookup[i & 0xf];
	} else {
		led_10.raw = LED_OFF; // Turn off led if zero (lose leading zeros)
	}
	if(value >= 10 || decimal || led_10.raw!=LED_OFF){ // If decimal, we want 1 leading zero
		for(i=0; value >= 10; i++){
			value -= 10;
		}
		led_1.raw = led_lookup[i];
		if(decimal){
			led_1.decimal = 1;
		}
	} else {
		led_1.raw = LED_OFF; // Turn off led if zero (lose leading zeros)
	}
	led_01.raw = led_lookup[(unsigned char)value];
}
#else
void value_to_led(int value, unsigned char decimal) {
	unsigned char i;
    
	// Handle negative values
	if (value < 0) {
		led_e.e_negative = 0;
		value = -value;
	} else {
		led_e.e_negative = 1;
	}

	// This assumes that only temperatures and all temperatures are decimal
	if(decimal){
		led_e.e_deg = 0;
#ifdef FAHRENHEIT
		led_e.e_c = 1;
#else
		led_e.e_c = 0;
#endif // FAHRENHEIT
	}
    
	// If temperature >= 100 we must lose decimal...
	if (value >= 1000) {
		value = divu10((unsigned int) value);
		decimal = 0;
	}

	// Convert value to BCD and set LED outputs
	if(value >= 100){
		for(i=0; value >= 100; i++){
			value -= 100;
		}
		led_10.raw = led_lookup[i & 0xf];
	} else {
		led_10.raw = LED_OFF; // Turn off led if zero (lose leading zeros)
	}
	if(value >= 10 || decimal || led_10.raw!=LED_OFF){ // If decimal, we want 1 leading zero
		for(i=0; value >= 10; i++){
			value -= 10;
		}
		led_1.raw = led_lookup[i];
		if(decimal){
			led_1.decimal = 0;
		}
	} else {
		led_1.raw = LED_OFF; // Turn off led if zero (lose leading zeros)
	}
	led_01.raw = led_lookup[(unsigned char)value];
}
#endif
/* To be called once every hour on the hour.
 * Updates EEPROM configuration when running profile.
 */
void update_profile(void){
	unsigned char profile_no = eeprom_read_config(EEADR_SET_MENU_ITEM(rn));

	// Running profile?
	if (profile_no < THERMOSTAT_MODE) {
		unsigned char curr_step = eeprom_read_config(EEADR_SET_MENU_ITEM(St));
		unsigned int curr_dur = eeprom_read_config(EEADR_SET_MENU_ITEM(dh)) + 1;
		unsigned char profile_step_eeaddr;
		unsigned int profile_step_dur;
		int profile_next_step_sp;

		// Sanity check
		if(curr_step > 8){
			curr_step = 8;
		}

		profile_step_eeaddr = EEADR_PROFILE_SETPOINT(profile_no, curr_step);
		profile_step_dur = eeprom_read_config(profile_step_eeaddr + 1);
		profile_next_step_sp = eeprom_read_config(profile_step_eeaddr + 2);

		// Reached end of step?
		if (curr_dur >= profile_step_dur) {
			// Update setpoint with value from next step
			eeprom_write_config(EEADR_SET_MENU_ITEM(SP), profile_next_step_sp);
			// Is this the last step (next step is number 9 or next step duration is 0)?
			if (curr_step == 8 || eeprom_read_config(profile_step_eeaddr + 3) == 0) {
				// Switch to thermostat mode.
				eeprom_write_config(EEADR_SET_MENU_ITEM(rn), THERMOSTAT_MODE);
				return; // Fastest way out...
			}
			// Reset duration
			curr_dur = 0;
			// Update step
			curr_step++;
			eeprom_write_config(EEADR_SET_MENU_ITEM(St), curr_step);
		} else if(eeprom_read_config(EEADR_SET_MENU_ITEM(rP))) { // Is ramping enabled?
			int profile_step_sp = eeprom_read_config(profile_step_eeaddr);
			unsigned int t = curr_dur << 6;
			long sp = 32;
			unsigned char i;

			// Linear interpolation calculation of new setpoint (64 substeps)
			for (i = 0; i < 64; i++) {
			 if (t >= profile_step_dur) {
			    t -= profile_step_dur;
			    sp += profile_next_step_sp;
			  } else {
			    sp += profile_step_sp;
			  }
			}
			sp >>= 6;

			// Update setpoint
			eeprom_write_config(EEADR_SET_MENU_ITEM(SP), sp);
		}
		// Update duration
		eeprom_write_config(EEADR_SET_MENU_ITEM(dh), curr_dur);
	}
}

/* Due to a fault in SDCC, static local variables are not initialized
 * properly, so the variables below were moved from temperature_control()
 * and made global.
 */
unsigned int cooling_delay = 60;  // Initial cooling delay
unsigned int heating_delay = 60;  // Initial heating delay

void temperature_control(void){
    
	int setpoint = eeprom_read_config(EEADR_SET_MENU_ITEM(SP));
#if defined PB2
	int hysteresis2 = eeprom_read_config(EEADR_SET_MENU_ITEM(hy2));
	unsigned char probe2 = eeprom_read_config(EEADR_SET_MENU_ITEM(Pb));
#endif

	if(cooling_delay){
		cooling_delay--;
	}
	if(heating_delay){
		heating_delay--;
	}

	// Set LED outputs
#if defined (__18F2520)
	led_e.e_cool = REL_COOL; // was lata4
	led_e.e_heat = REL_HEAT;
#else
	led_e.e_cool = !REL_COOL; // was lata4
	led_e.e_heat = !REL_HEAT;    
#endif
	// This is the thermostat logic
#if defined PB2
	if((REL_COOL && (temperature <= setpoint || (probe2 && (temperature2 < (setpoint - hysteresis2))))) || (REL_HEAT && (temperature >= setpoint || (probe2 && (temperature2 > (setpoint + hysteresis2)))))){
#else
	if((REL_COOL && (temperature <= setpoint )) || (REL_HEAT && (temperature >= setpoint))){
#endif
		cooling_delay = eeprom_read_config(EEADR_SET_MENU_ITEM(cd)) << 6;
		cooling_delay = cooling_delay - (cooling_delay >> 4);
		heating_delay = eeprom_read_config(EEADR_SET_MENU_ITEM(hd)) << 6;
		heating_delay = heating_delay - (heating_delay >> 4);
		REL_COOL = 0;
		REL_HEAT = 0;
	}
	else if(REL_COOL == 0 && REL_HEAT == 0) {
		int hysteresis = eeprom_read_config(EEADR_SET_MENU_ITEM(hy));
#ifdef PB2
		hysteresis2 >>= 2; // Halve hysteresis 2
		if ((temperature > setpoint + hysteresis) && (!probe2 || (temperature2 >= setpoint - hysteresis2))) {
#else
		if (temperature > setpoint + hysteresis) {
#endif
			if (cooling_delay) {
				led_e.e_cool = led_e.e_cool ^ (cooling_delay & 0x1); // Flash to indicate cooling delay
			} else {
				REL_COOL = 1;
			}
#if defined PB2
		} else if ((temperature < setpoint - hysteresis) && (!probe2 || (temperature2 <= setpoint + hysteresis2))) {
#else
		} else if (temperature < setpoint - hysteresis) {
#endif
			if (heating_delay) {
				led_e.e_heat = led_e.e_heat ^ (heating_delay & 0x1); // Flash to indicate heating delay
			} else {
				REL_HEAT = 1;
			}
		}
	}
}

    
/*******************************************************************************
 *  Initialize hardware etc, on startup.
 * arguments: none
 * returns: nothing
 */
void init() {
    int i;
    
    ADCON1 |= 0x0F;                 // Default all pins to digital
	OSCCON = 0b01101010; // 4MHz

	// Heat, cool as output, Thermistor as input, piezo output
#if defined FO433
	TRISA = 0b00001100;
#else
	SENS_TRIS=0b10100011; // alternative with RA5 and RA7 as KEB input (free RB6/7 for debug)
//	SENS_TRIS=0b00000011;
#endif
	SENS_PORT= 0; // Drive relays and piezo low

	// LED Common cathode C4-C7
	DIG_PORT = 0xF0;
	DIG_TRIS = 0b00001111;

	// LED data  output
    LED_PORT = 0x00;
	LED_TRIS = 0; // led 0-7

   	// Analog input on thermistor
    ADCON1=0b00001101; // vref = vcc, RA0-RA1 as sensor input
            
	// AD clock FOSC/8 (FOSC = 4MHz)
	ADCON2bits.ADCS0=1; //0b100;

    // AD acq time 8TAD
    ADCON2bits.ACQT=0b100; 

    
	// Right justify AD result
	ADCON2bits.ADFM = 1;

	// IMPORTANT FOR BUTTONS TO WORK!!! Disable analog input -> enables digital input
	// ???????????? not required for PIC18F ANSELC = 0;

	// Postscaler 1:1, Enable counter, prescaler 1:4
    T2CON=0;
    T2CONbits.T2CKPS=1;
	// @4MHz, Timer 2 clock is FOSC/4 -> 1MHz prescale 1:4-> 250kHz, 250 gives interrupt every 1 ms
	PR2 = 250;
	// Enable Timer2 interrupt
    T2CONbits.TMR2ON=1;
    IPR1bits.TMR2IP=1; // high priority
	PIE1bits.TMR2IE = 1;
    
#if defined (__P16F2428) 
	// Postscaler 1:15, - , prescaler 1:16
	T4CON = 0b01110010;
	TMR4ON = eeprom_read_config(EEADR_POWER_ON);
	// @4MHz, Timer 2 clock is FOSC/4 -> 1MHz prescale 1:16-> 62.5kHz, 250 and postscale 1:15 -> 16.66666 Hz or 60ms
	PR4 = 250;

	// Postscaler 1:7, Enable counter, prescaler 1:64
	T6CON = 0b00110111;
	// @4MHz, Timer 2 clock is FOSC/4 -> 1MHz prescale 1:64-> 15.625kHz, 250 and postscale 1:6 -> 8.93Hz or 112ms
	PR6 = 250;
#elif defined(__18F2520)
    // TMR4ON is flag in software
	TMR4ON = eeprom_read_config(EEADR_POWER_ON);
    vPR6 = 112;
#endif
    
    
#if defined COM
	// Set PEIE and GIE (enable global interrupts), IOCIE
	INTCON = 0b11001000;
	IOCAP = 2;	// Rising edge detect
	IOCAN = 0;
#else
	// Set PEIE (enable peripheral interrupts, that is for timer2) and GIE (enable global interrupts)

    RCONbits.IPEN = 1;            //enable priority levels
    INTCONbits.GIE=1;
    INTCONbits.GIEH=1;
    
#endif

#if defined FO433
	// Timer 0 prescale 8
	PSA=0;
	PS0=0;
	PS1=1;
	PS2=0;
#endif

}

/* Interrupt service routine.
 * Receives timer2 interrupts every millisecond.
 * Handles multiplexing of the LEDs.
 */

void high_isr(void);
/*********************High priority interrupt vector **************************/
#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
  _asm GOTO high_isr _endasm
}

#pragma code
/*******************************High priority ISR *****************************/
#pragma interrupt high_isr
void high_isr (void)
{
static unsigned char latb=0;
#ifdef COM
	if(IOCAF1){

		/* Disable pin change interrupt and edge detection */
		IOCIE = 0;
		IOCAP1 = 0;

		/* If sending a '1' bit */
		if(com_write && (com_data & 0x80)){
			TRISA1 = 0;
			LATA1 = 1;
		}

		/* Init communication reset countdown (10ms) */
		com_tmout = 10;

		/* Enable timer 0 to generate interrupt for reading/writing in 250us */
		TMR0 = 5;
		TMR0CS = 0;
		TMR0IE = 1;

		/* Clear edge detection flag */
		IOCAF = 0;	
	}

	if(TMR0IF){
		/* Disable timer 0 */
		TMR0CS = 1;
		TMR0IE = 0;

		/* Make sure RA1 is input */
		TRISA1 = 1;
		LATA1 = 0;

		/* Shift data */
		com_data <<= 1;
		com_count++;

		/* If receive '1' */
		if(!com_write && RA1){
			com_data |= 1;
		}

		/* Enable edge detection and pin change interrupt */
		IOCAF = 0;
		IOCAP1 = 1;
		IOCIE = 1;

		/* Clear timer 0 interrupt flag */
		TMR0IF = 0;
	}
#elif defined FO433
	if(TMR0IF){
		/* */
		LATA1 = 0;

		if(fo433_send_space){
			fo433_count>>=1;
			TMR0 = 130;
			fo433_send_space = 0;
		} else {
			if(fo433_data & fo433_count){
				TMR0 = 193;
			} else {
				TMR0 = 67;
			}
			LATA1 = 1;
			fo433_send_space = 1;
		}

		if(fo433_count == 0){
			/* Disable timer 0 */
			TMR0CS = 1;
			TMR0IE = 0;
		}

		/* Clear timer 0 interrupt flag */
		TMR0IF = 0;
	}
#endif
	// Check for Timer 2 interrupt (every 1ms)
	// Kind of excessive when it's the only enabled interrupt
	// but is nice as reference if more interrupts should be needed
	if (PIR1bits.TMR2IF) {
#if defined (__16F2626)
        unsigned char latb = (DIG_PORT << 1);
#elif defined (__18F2520)
        
        // lat b used for common cathode, inverted when writing to LED
        latb<<=1;
#endif
        
		if(latb == 0){
			latb = 0x10;
		}
        
                
		LED_TRIS = LEDOUT; // Ensure LED data pins are outputs
		LED_PORT = LEDOFF; // Disable LED's while switching

		// Multiplex LED's every millisecond
		switch(latb) {
			case 0x10:
			LED_PORT = led_01.raw;
			break;
			case 0x20:
			LED_PORT = led_10.raw;
			break;
			case 0x40: 
			LED_PORT = led_1.raw;
			break;
			case 0x80:
			LED_PORT = led_e.raw;
			break;
		}

		// Enable new LED Digit
        // as cathode on digit, we reverse the bits
		DIG_PORT = ~latb;

#if defined COM
		/* Reset communication state on timeout */
		if(com_tmout){
			com_tmout--;
		} else {
			com_state = 0;
			com_count = 0;
			com_write = 0;
		}
#endif // COM

#if defined (__18F2520)

// software implementation of time 4 and 6 not available in 18f2520 hardware
//            if(tmr6cnt++== TMR6_MAX){   // 112ms
            if(tmr6cnt++ >= vPR6){   // 112ms
                tmr6cnt=0;
                TMR6IF=1;
            }

        if(TMR4ON){
            
            if(tmr4cnt++ == TMR4_MAX){  // 60ms
                tmr4cnt=0;
                TMR4IF=1;
            };
        }
        
#endif
        
		// Clear interrupt flag
		PIR1bits.TMR2IF = 0;
	}
    
} // end ISR

#define _ADON 1
#define _CHS0 2
#define _CHS1 4

#define START_TCONV_1()		(ADCON0 = _CHS0 | _ADON)
#define START_TCONV_2()		(ADCON0 = _CHS1 | _ADON)

/******************************************************************************/
unsigned int read_ad(unsigned int adfilter){

    static unsigned long temp;
	ADCON0bits.GO = 1;
	while(ADCON0bits.GO);
//	ADCON0bits.ADON = 0;
    temp =((adfilter - (adfilter >> 6)) + ((ADRESH << 8) | ADRESL));
	return (unsigned int)temp;

}

/******************************************************************************/
int ad_to_temp(unsigned int adfilter){
	unsigned char i;
    long temp = 32;
	unsigned char a = ((adfilter >> 5) & 0x3f); // Lower 6 bits
	unsigned char b = ((adfilter >> 11) & 0x1f); // Upper 5 bits

	// Interpolate between lookup table points
    
    // Table is 32 words long = 64 bytes
	for (i = 0; i < SAMPLES; i++) {
		if(a <= i) {
			temp += ad_lookup[b];
		} else {
			temp += ad_lookup[b + 1];
		}
	}

	// Divide by 64 to get back to normal temperature
	return (temp >> 6);
}

/******************************************************************************/
    int ad_to_temp2(unsigned int adfilter){

#define MINTABLE -250
#define TABLESTEP 50 // 1/10 of degree
    
    int temp = MINTABLE; // in 1/10 of deg.
    char tabPtr=0;
    int gapPos, delta=0;
    int level;
    level=(int)(adfilter/SAMPLES)-512;
    
    while((ad_lookup[tabPtr] < (level) )&& (tabPtr<32))
        {
            temp +=TABLESTEP; // we increment by 5 deg at each step
            tabPtr++;
        }
    
    // calculate gap in adc value between 2 points(5 deg). 
    delta=ad_lookup[tabPtr]-ad_lookup[tabPtr-1];
    
    // calculate probe position within the gap 
    gapPos=adfilter/SAMPLES-ad_lookup[tabPtr-1];
    
    // add the gap to the lookup value
    
    temp+=(delta*TABLESTEP/gapPos);
    
	// Divide by 64 to get back to normal temperature
	return (temp);
}



#if defined COM
enum com_states {
	com_idle = 0,
	com_recv_addr,
	com_recv_data1,
	com_recv_data2,
	com_recv_checksum,
	com_trans_data1,
	com_trans_data2,
	com_trans_checksum,
	com_trans_ack
};

/* State machine to handle rx/tx protocol */
static void handle_com(unsigned const char rxdata){
	static unsigned char command;
	static unsigned char xorsum;
	static unsigned int data;
	static unsigned char addr;

	xorsum ^= rxdata;

	com_write = 0;

	if(com_state == com_idle){
		command = rxdata;
		xorsum = rxdata;
		addr = 0;
		if(command == COM_READ_EEPROM || command == COM_WRITE_EEPROM){
			com_state = com_recv_addr;
		} else if(rxdata == COM_READ_TEMP){
			data = temperature;
			com_state = com_trans_data1;
		} else if(rxdata == COM_READ_COOLING){
			data = REL_COOL;
			com_state = com_trans_data1;
		} else if(rxdata == COM_READ_HEATING){
			data = REL_HEAT;
			com_state = com_trans_data1;
		}
	} else if(com_state == com_recv_addr){
		addr = rxdata;
		if(command == COM_WRITE_EEPROM){
			com_state = com_recv_data1;
		} else {
			data = (unsigned int)eeprom_read_config(addr);
			com_state = com_trans_data1;
		}			
	} else if(com_state == com_recv_data1 || com_state == com_recv_data2){
		data = (data << 8) | rxdata;
		com_state++;
	} else if(com_state == com_recv_checksum){
		if(xorsum == 0){
			eeprom_write_config(addr, (int)data);
			com_data = COM_ACK;
		} else {
			com_data = COM_NACK;
		}
		com_write = 1;
		com_state = com_idle;
	} else if(com_state == com_trans_data2){
		com_data = (unsigned char) data; 
		com_write = 1;
		com_state = com_trans_checksum;
	} else if(com_state == com_trans_checksum){
		com_data = command ^ addr ^ ((unsigned char)(data >> 8)) ^ ((unsigned char) data);
		com_write = 1;
		com_state = com_trans_ack;
	} else if(com_state == com_trans_ack){
		com_data = COM_ACK;
		com_write = 1;
		com_state = com_idle;
	}

	if(com_state == com_trans_data1 || com_state == com_trans_data2){
		com_data = (data >> 8);
		com_write = 1;
		com_state++;
	}

}

#elif defined FO433

enum fo433_states {
	fo433_preamble=0,
	fo433_devid_high,
	fo433_devid_low_temp_high,
	fo433_temp_low,
	fo433_humidity,
	fo433_crc
};

/* Fine offset state machine */
static void fo433_fsm(){
	static int t;
	static unsigned char crc;

	if(fo433_state == fo433_preamble){
		fo433_data = 0xff;
	} else if(fo433_state == fo433_devid_high){
		fo433_data = 0x45;
		crc = 0;
	} else if(fo433_state == fo433_devid_low_temp_high){
		unsigned char di = ((unsigned char)eeprom_read_config(EEADR_SET_MENU_ITEM(hy2))) << 4;
		t = temperature;
		fo433_data = (di | ((t >> 8) & 0xf));
	} else if(fo433_state == fo433_temp_low){
		fo433_data = (unsigned char) t;
	} else if(fo433_state == fo433_humidity){
		fo433_data = (REL_HEAT << 6) | (REL_COOL << 4);
	} else if(fo433_state == fo433_crc){
		fo433_data = crc;
	}

	// Calc CRC
	{
		unsigned char i=8;
		unsigned char d = fo433_data;
		while(i-- > 0){
			unsigned char mix = (crc ^ d) & 0x80;
			crc <<= 1;
			if(mix){
				crc ^= 0x31;
			}
			d <<= 1;
		}
	}

	fo433_count = 0x80;

	/* Enable timer 0 */
	TMR0IE = 1;
	TMR0CS = 0;
}

#endif

unsigned int ad_filter;
unsigned int millisx60=0;
/******************************************************************************
 * Main entry point.
 */
    
void main(void)  {
    
//  static unsigned int millisx60=0;
//	static unsigned int ad_filter=0x7ffe; // equals to 512*64 samples = around 0
//	static unsigned int ad_filter;
    char i;
#if defined PB2
	unsigned int ad_filter2=0x7ffe;
#endif

	init();
    
 	START_TCONV_1();   
    
    //    init_adc buffer
    for(i=0;i<SAMPLES;i++){
        ad_filter=read_ad(ad_filter);
    }
    


#ifdef TEST_1DIGIT
    while(1){
				led_e.e_negative = 1;
				led_10.raw = LED_OFF;
				led_1.raw = led_01.raw = LED_OFF;
    }
#endif
	//Loop forever
	while (1) {

#if defined COM
		/* Handle communication if full byte transmitted/received */
		GIE = 0;
		if(com_count >= 8){
			char rxdata = com_data;
			com_count = 0;
			GIE = 1;
 			handle_com(rxdata);
		}
		GIE = 1;
#elif defined FO433
		/* Send next byte */
		if((fo433_state <= fo433_crc) && (fo433_count == 0)){
			fo433_fsm();
			fo433_state++;
		}
#endif
          
		if(TMR6IF) { 

			// Handle button press and menu
			button_menu_fsm();

			if(!TMR4ON){
				led_e.raw = LED_OFF;
				led_10.raw = LED_O;
				led_1.raw = led_01.raw = LED_F;
			}

			// Reset timer flag
			TMR6IF = 0;
		}

		if(TMR4IF) {

			millisx60++;

			if(millisx60 & 0x1){
				ad_filter = read_ad(ad_filter);

#if defined PB2
				START_TCONV_2();
			} else {
				ad_filter2 = read_ad(ad_filter2);
#endif
				START_TCONV_1();
			}

			// Only run every 16th time called, that is 16x60ms = 960ms
			// Close enough to 1s for our purposes.
			if((millisx60 & 0xf) == 0) {

				temperature = ad_to_temp(ad_filter) + eeprom_read_config(EEADR_SET_MENU_ITEM(tc));
//				temperature = ad_to_temp(ad_filter) +TCORRECTION+ eeprom_read_config(EEADR_SET_MENU_ITEM(tc));
#if defined PB2
				temperature2 = ad_to_temp(ad_filter2) + eeprom_read_config(EEADR_SET_MENU_ITEM(tc2));
#endif

#if defined FO433
				if(fo433_sec_count < 3){
					fo433_state = 0;
				}
				if(fo433_sec_count == 0){
					fo433_sec_count = 48;
				}
				fo433_sec_count--;
			
#endif

				// Alarm on sensor error (AD result out of range)
				BUZ = ((ad_filter>>8) >= 248 || (ad_filter>>8) <= 8) 
#if defined PB2
					 (eeprom_read_config(EEADR_SET_MENU_ITEM(Pb)) && ((ad_filter2>>8) >= 248 || (ad_filter2>>8) <= 8))
#endif
				;

				if(BUZ){ // On alarm, disable outputs
					led_10.raw = LED_A;
					led_1.raw = LED_L;
					led_e.raw = led_01.raw = LED_OFF;
					REL_COOL = 0;
					REL_HEAT = 0;
					cooling_delay = heating_delay = 60;
				} else {
					// Update running profile every hour (if there is one)
					// and handle reset of millis x60 counter
					if(((unsigned char)eeprom_read_config(EEADR_SET_MENU_ITEM(rn))) < THERMOSTAT_MODE){
						// Indicate profile mode
						led_e.e_set = 0;
						// Update profile every hour
						if(millisx60 >= 60000){
							update_profile();
							millisx60 = 0;
						}
					} else {
						led_e.e_set = 1;
						millisx60 = 0;
					}

					{
						int sa = eeprom_read_config(EEADR_SET_MENU_ITEM(SA));
						if(sa){
							int diff = temperature - eeprom_read_config(EEADR_SET_MENU_ITEM(SP));
							if(diff < 0){
								diff = -diff;
							} 
							if(sa < 0){
								sa = -sa;
								BUZ = diff <= sa;
							} else {
								BUZ = diff >= sa;
							}
						}
					}

					// Run thermostat
					temperature_control();

					// Show temperature if menu is idle
					if(menuON){ // was TMR1GE
						if(BUZ && alarmFlash){ //RX9 ){ 
							led_10.raw = LED_S;
							led_1.raw = LED_A;
							led_01.raw = LED_OFF;
						} else {
#if defined PB2
							led_e.e_point = !memu01; // was TX9
							if(menu01){ // was TX9
								temperature_to_led(temperature2);
							} else {
								temperature_to_led(temperature);
							}
#else
							temperature_to_led(temperature);
#endif
						}
//						RX9 = !RX9;
                        alarmFlash!=alarmFlash;
					}
				}
			} // End 1 sec section
            
        	// Reset timer flag
			TMR4IF = 0;
		}
		// Reset watchdog
		ClrWdt();
	} // end while(1)
} // end main

