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
 * Schematic of the connections to the MCU.
 *
 *                                     PIC16F1828
 *                                    ------------
 *                                VDD | 1     20 | VSS
 *                     Relay Heat RA5 | 2     19 | RA0/ICSPDAT (Programming header), Piezo buzzer
 *                     Relay Cool RA4 | 3     18 | RA1/AN1/ICSPCLK (Programming header), Thermistor
 * (Programming header) nMCLR/VPP/RA3 | 4     17 | RA2/AN2 Thermistor
 *                          LED 5 RC5 | 5     16 | RC0 LED 0
 *                   LED 4, BTN 4 RC4 | 6     15 | RC1 LED 1
 *                   LED 3, BTN 3 RC3 | 7     14 | RC2 LED 2
 *                   LED 6, BTN 2 RC6 | 8     13 | RB4 LED Common Anode 10's digit
 *                   LED 7, BTN 1 RC7 | 9     12 | RB5 LED Common Anode 1's digit
 *        LED Common Anode extras RB7 | 10    11 | RB6 LED Common Anode 0.1's digit
 *                                    ------------
 *
 *
 * Schematic of the bit numbers for the display LED's. Useful if custom characters are needed.
 *
 *           * 7       --------    *    --------       * C
 *                    /   7   /    1   /   7   /       5 2
 *                 2 /       / 6    2 /       / 6    ----
 *                   -------          -------     2 / 7 / 6
 *           *     /   1   /        /   1   /       ---
 *           3  5 /       / 3    5 /       / 3  5 / 1 / 3
 *                --------    *    --------   *   ----  *
 *                  4         0      4        0    4    0
 *
 *  
 *                                     PIC18F2520
 *                                    ------------
 *                              MCLR_ | 1     28 | RB7 LED7/KB-SET/PGD
 *                          SENS2 RA0 | 2     27 | RB6 LED6/KB-DWN/PGC
 *                          SENS1 RA1 | 3     26 | RB5 LED5/KB-PWR
 *                          REL1  RA2 | 4     25 | RB4 LED4/KB-UP
 *                          REL2  RA3 | 5     24 | RB3 LED3
 *                          BUZ   RA4 | 6     23 | RB2 LED3
 *                          NC    RA5 | 7     22 | RB1 LED1
 *                                VSS | 8     21 | RB0 LED0
 *                          NC    RA7 | 9     20 | VDD 
 *                          REL3  RA6 | 10    19 | VSS
 *                    32khz OSC1  RC0 | 11    18 | RC7 LED Common cathode sign digit
 *                          OSC2  RC1 | 12    17 | RC6 LED Common cathode 0.1's digit
 *                          IR    RC2 | 13    16 | RC5 LED Common cathode 1.0's digit
 *                          NC    RC3 | 14    15 | RC4 LED Common cathode 10.0's digit
 *                                    ------------
 *          IR sensor and 32khz oscillator not yet implemented
 * 
 *          PIC18f2520 version common cathode, leds have different layout !!!!
 * 
 *              * 5    --------  
 *                    /   0   /  
 *                 5 /       / 1 
 *                   -------     
 *           *     /   6   /     
 *           4  4 /       / 2    
 *                --------    *   
 *                  3         7   
 * 
 *
 */

#ifndef __STC1000P_H__
#define __STC1000P_H__


// IO definitions added by Rolf Ziegler June 2015
//#define PB2
#if defined  (__18F2520)
#define REL_HEAT     LATAbits.LATA2
#define REL_COOL     LATAbits.LATA3
#define BUZ      LATAbits.LATA4
#define SENS1    PORTAbits.RA0
#define SENS2    PORTAbits.RA1
#define SENS_TRIS TRISA
#define SENS_PORT LATA

#define LED0    LATBbits.LATB0
#define LED1    LATBbits.LATB1
#define LED2    LATBbits.LATB2
#define LED3    LATBbits.LATB3
#define LED4    LATBbits.LATB4
#define LED5    LATBbits.LATB5
#define LED6    LATBbits.LATB6
#define LED7    LATBbits.LATB7
#define LED_PORT  LATB
#define LED_TRIS TRISB

#define LEDOUT 0x00
#define LEDOFF 0x00

#define DIG_PORT LATC // C4-7
#define DIG_TRIS TRISC

#define KB_UP      !PORTBbits.RB4
#define KB_PWR     !PORTBbits.RB5
//#define KB_DWN     PORTBbits.RB6 // moved for debugging reasons 
#define KB_DWN    !PORTAbits.RA7 
//#define KB_SET    !PORTBbits.RB7 // B6 and B7 are used by pickit2 and ICD3 for debugging
#define KB_SET    !PORTAbits.RA5
#define KB_PORT     PORTB
#define KB_TRIS     TRISB

#elif defined (__pic16F1828) // pic18 compiler not compatible, not tested
#define SENS1        PORTAbits.RA1
#define SENS2        PORTAbits.RA2
#define REL_HEAT     LATAbits.LATA5 // A3 does not exist
#define REL_COOL     LATAbits.LATA4
#define BUZ          LATAbits.LATA0
#define SENS_TRIS    TRISA
#define SENS_PORT    LATA

#define LED0    LATCbits.LATC0
#define LED1    LATCbits.LATC1
#define LED2    LATCbits.LATC2
#define LED3    LATCbits.LATC4
#define LED5    LATCbits.LATC5
#define LED6    LATCbits.LATC6
#define LED7    LATCbits.LATC7
#define LED_PORT  LATC
#define LED_TRIS TRISC

#define LEDOUT 0x00
#define LEDOFF 0x00

#define DIG_PORT LATB // C4-7
#define DIG_TRIS TRISB

#define KB_UP      PORTCbits.RC4
#define KB_PWR     PORTCbits.RC5
#define KB_DWN     PORTCbits.RC6
#define KB_SET     PORTCbits.RC7
#define KB_PORT     PORTC
#define KB_TRIS     TRISC

#else
#error PROCESSOR/PIC NOT SUPPORTED
#endif

/* Define STC-1000+ version number (XYY, X=major, YY=minor) */
/* Also, keep track of last version that has changes in EEPROM layout */
#define STC1000P_VERSION		108
#define STC1000P_EEPROM_VERSION		11

/* Define limits for temperatures */
#ifdef FAHRENHEIT
#define TEMP_MAX		(2500)
#define TEMP_MIN		(-400)
#define TEMP_CORR_MAX		(100)
#define TEMP_CORR_MIN		(-100)
#define TEMP_HYST_1_MAX		(100)
#define TEMP_HYST_2_MAX		(500)
#define SP_ALARM_MIN		(-800)
#define SP_ALARM_MAX		(800)
#else  // CELSIUS
#define TEMP_MAX            (1400)
#define TEMP_MIN            (-400)
#define TEMP_CORR_MAX		(800) // changed by rz
#define TEMP_CORR_MIN		(-800)// changed by rz
#define TEMP_HYST_1_MAX		(50)
#define TEMP_HYST_2_MAX		(250)
#define SP_ALARM_MIN		(-400)
#define SP_ALARM_MAX		(400)
#endif

/* The data needed for the 'Set' menu
 * Using x macros to generate the data structures needed, all menu configuration can be kept in this
 * single place.
 *
 * The values are:
 * 	name, LED data 10, LED data 1, LED data 01, min value, max value, default value celsius, default value fahrenheit
 */
#define SET_MENU_DATA(_) \
    _(hy, 	LED_h, 	LED_y, 	LED_OFF, 	0, 		TEMP_HYST_1_MAX,	5,		10) 	\
    _(hy2, 	LED_h, 	LED_y, 	LED_2, 		0, 		TEMP_HYST_2_MAX, 	50,		100)	\
    _(tc, 	LED_t, 	LED_c, 	LED_OFF, 	TEMP_CORR_MIN, 	TEMP_CORR_MAX,		0,		0)	\
    _(tc2, 	LED_t, 	LED_c, 	LED_2, 		TEMP_CORR_MIN,	TEMP_CORR_MAX,		0,		0)	\
    _(SA, 	LED_S, 	LED_A, 	LED_OFF, 	SP_ALARM_MIN,	SP_ALARM_MAX,		0,		0)	\
    _(SP, 	LED_S, 	LED_P, 	LED_OFF, 	TEMP_MIN,	TEMP_MAX,		200,		680)	\
    _(St, 	LED_S, 	LED_t, 	LED_OFF, 	0,		8,			0,		0)	\
    _(dh, 	LED_d, 	LED_h, 	LED_OFF, 	0,		999,			0,		0)	\
    _(cd, 	LED_c, 	LED_d, 	LED_OFF, 	0,		60,			5,		5)	\
    _(hd, 	LED_h, 	LED_d, 	LED_OFF, 	0,		60,			2,		2)	\
    _(rP, 	LED_r, 	LED_P, 	LED_OFF, 	0,		1,			0,		0)	\
    _(Pb, 	LED_P, 	LED_b, 	LED_2, 		0,		1,			0,		0)	\
    _(rn, 	LED_r, 	LED_n, 	LED_OFF, 	0,		6,			6,		6) 	\

#define ENUM_VALUES(name, led10ch, led1ch, led01ch, minv, maxv, dvc, dvf) \
    name,

/* Generate enum values for each entry int the set menu */
enum set_menu_enum {
    SET_MENU_DATA(ENUM_VALUES)
};

#define NO_OF_PROFILES			6
#define SET_MENU_ITEM_NO		NO_OF_PROFILES
#define THERMOSTAT_MODE			NO_OF_PROFILES

/* Defines for EEPROM config addresses */
#define EEADR_PROFILE_SETPOINT(profile, step)	(((profile)*19) + ((step)<<1))
#define EEADR_PROFILE_DURATION(profile, step)	EEADR_PROFILE_SETPOINT(profile, step) + 1
#define EEADR_SET_MENU				EEADR_PROFILE_SETPOINT(NO_OF_PROFILES, 0)
#define EEADR_SET_MENU_ITEM(name)		(EEADR_SET_MENU + (name))
#define EEADR_POWER_ON				127

#define SET_MENU_SIZE				(sizeof(setmenu)/sizeof(setmenu[0]))

#if defined (__16F1828) 
#define LED_OFF	0xff
#define LED_0	0x3
#define LED_1	0xb7
#define LED_2	0xd
#define LED_3	0x25
#define LED_4	0xb1
#define LED_5	0x61
#define LED_6	0x41
#define LED_7	0x37
#define LED_8	0x1
#define LED_9	0x21
#define LED_A	0x11
#define LED_a	0x5
#define LED_b	0xc1
#define LED_C	0x4b
#define LED_c	0xcd
#define LED_d	0x85
#define LED_e	0x9
#define LED_E	0x49
#define LED_F	0x59
#define LED_H	0x91
#define LED_h	0xd1
#define LED_I	0xb7
#define LED_J	0x87
#define LED_L	0xcb
#define LED_n	0xd5	
#define LED_O	0x3
#define LED_P	0x19
#define LED_r	0xdd	
#define LED_S	0x61
#define LED_t	0xc9
#define LED_U	0x83
#define LED_y	0xa1
#elif defined (__18F2520)
// LED_PORT = Anode, segments, 8 bits
// DIG_PORT = Cathode, digit 4 bits
#define LED_OFF	0b00000000
#define LED_0	0b00111111
#define LED_1	0b00000110
#define LED_2	0b01011011
#define LED_3	0b01001111
#define LED_4	0b01100110
#define LED_5	0b01101101
#define LED_6	0b01111100
#define LED_7	0b00000111
#define LED_8	0b01111111
#define LED_9	0b01100111
#define LED_A	0b01110111
#define LED_a	0b01011100
#define LED_b	0b01111100
#define LED_C	0b00111001
#define LED_c	0b01011000
#define LED_d	0b01011110
#define LED_e	0b01111001
#define LED_E	0b01111001
#define LED_F	0b01110001
#define LED_H	0b01110110
#define LED_h	0b01110100
#define LED_I	0b00000110
#define LED_J	0b00001110
#define LED_L	0b00111000
#define LED_n	0b01010100
#define LED_O	0b00111111
#define LED_P	0b01110011
#define LED_r	0b01010000
#define LED_S	0b01101101
#define LED_t	0b01111000
#define LED_U	0b00111110
#define LED_y	0b01101110
#endif
/* Declare functions and variables from Page 0 */

#define COM_READ_EEPROM		0x20
#define COM_WRITE_EEPROM	0xE0
#define COM_READ_TEMP		0x01
#define COM_READ_COOLING	0x02
#define COM_READ_HEATING	0x03
#define COM_ACK             0x9A
#define COM_NACK            0x66
   
#if defined (__16f1828) 
typedef union
{
	unsigned char raw;

	struct
	  {
	  unsigned                      : 1;
	  unsigned e_point              : 1;
	  unsigned e_c                  : 1;
	  unsigned e_heat               : 1;
	  unsigned e_negative           : 1;
	  unsigned e_deg                : 1;
	  unsigned e_set                : 1;
	  unsigned e_cool               : 1;
	  };
} led_e_t;
typedef union
{
	unsigned char raw;

	struct
	  {
	  unsigned decimal		: 1;
	  unsigned middle		: 1;
	  unsigned upper_left		: 1;
	  unsigned lower_right          : 1;
	  unsigned bottom		: 1;
	  unsigned lower_left		: 1;
	  unsigned upper_right		: 1;
      unsigned top : 1;
    };
} led_t;


#elif defined (__18F2520)
typedef union
{
	unsigned char raw;

	struct
	  {
	  unsigned e_set                : 1; // dot
	  unsigned e_c1                 : 1; // upper one
   	  unsigned e_c                  : 1; // lower one
	  unsigned e_deg                : 1; // low point
	  unsigned e_point              : 1; // mid point
	  unsigned e_cool               : 1; // cooling sign
	  unsigned e_negative           : 1; // negative sign
   	  unsigned e_heat               : 1; // heating sign
	  };
} led_e_t;


typedef union
{
	unsigned char raw;

	struct
	  {
	  unsigned middle		: 1;
	  unsigned upper_left		: 1;
	  unsigned lower_right          : 1;
	  unsigned bottom		: 1;
	  unsigned lower_left		: 1;
	  unsigned upper_right		: 1;
      unsigned top : 1;
   	  unsigned decimal		: 1;
	  };
} led_t;

typedef struct
    {
    unsigned inmenu :1;
    unsigned temp1  :1;
    unsigned binter :1;
    unsigned :1;
    unsigned :1;
    unsigned :1;
    unsigned :1;
    unsigned :1;
    }flag_t;
#endif

extern led_e_t led_e;
extern led_t led_10, led_1, led_01;
extern unsigned const char led_lookup[];

extern char menu01;

extern unsigned int eeprom_read_config(unsigned char eeprom_address);
extern void eeprom_write_config(unsigned char eeprom_address,unsigned int data);
extern void value_to_led(int value, unsigned char decimal);
#define int_to_led(v)		value_to_led(v, 0)
#define temperature_to_led(v)	value_to_led(v, 1)

/* Declare functions and variables from Page 1 */
 void button_menu_fsm(void);
 void init(void);
 void update_profile(void);
 void temperature_control(void);
 
extern unsigned char TMR4IF;
extern unsigned char TMR4ON;

extern char vPR6;
#endif // __STC1000P_H__
