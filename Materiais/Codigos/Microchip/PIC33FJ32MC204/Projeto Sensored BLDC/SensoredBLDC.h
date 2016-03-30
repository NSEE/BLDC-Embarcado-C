//---------------------------------------------------------------------
//
//							 Software License Agreement
//
// The software supplied herewith by Microchip Technology Incorporated 
// (the “Company”) for its PICmicro® Microcontroller is intended and 
// supplied to you, the Company’s customer, for use solely and 
// exclusively on Microchip PICmicro Microcontroller products. The 
// software is owned by the Company and/or its supplier, and is 
// protected under applicable copyright laws. All rights are reserved. 
//  Any use in violation of the foregoing restrictions may subject the 
// user to criminal sanctions under applicable laws, as well as to 
// civil liability for the breach of the terms and conditions of this 
// license.
//
// THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES, 
// WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
// TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
// PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
// IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
//---------------------------------------------------------------------
//	File:		SensoredBLDC.h
//
//						
// 
// The following files should be included in the MPLAB project:
//
//		SensoredBLDC.c		-- Main source code file
//		Interrupts.c
//		Init.c
//		SensoredBLDC.h		-- Header file
//		p33FJ32MC204.gld	-- Linker script file
//				
//
//---------------------------------------------------------------------- 

/*
Using the Fosc = Fin * (M/(N1*N2))
Where:
Fin 	= 8Mhz
M 	= PLLFBD[PLLDIV<8:0>] 	= 0x26	then M = 40  (PLLDIV + 2)
N1	= CLKDIV[PLLPRE<4:0>] 	= 0x0 	then N1 = 2  (PLLPRE + 2)
N2	= CLKDIV[PLLPOST<1:0>]	= 0x3	then N2 = 8 

CLKDIV = 0x30C0
PLLFBD = 0x0026

?	Fosc =	(8M * (40 / (8 * 2)))  = 20Mhz
?	Fcy  =	20M/2 = 10MIP
*/

#define CLOSEDLOOP
#define FOSC  20000000			// xtal = 8.0Mhz, 20.0Mhz after PLL
#define FCY  FOSC/2				
#define MILLISEC FCY/20000		// 1 mSec delay constant
#define FPWM 39000

#define POLEPAIRS		5		// number of pole pairs
#define HALL_INDEX_R	4		// Hall sensor position index
#define HALL_INDEX_F	5		// Hall sensor position index

#define S2	PORTAbits.RA8
#define S3	PORTBbits.RB4

#define T1PR1 ((FCY/1000)/64)

/* Based on using the internal Fcy and Timer 3 prescaler of 256
 * Fcy/256 = 10M/256 = 39062.5 ticks/sec
 * or, 2343750 ticks = 1RPM
 * => RPM would be 2343750/T3ticks
 */
#define SPEEDMULT	2343750
#define OFFSET 8
#define POTMULT 4				// pot to speed ratio

//#define Kps	50					// Kp and Ks terms need to be adjusted
//#define Kis	1					// as per the motor and load 

struct MotorFlags
{
unsigned RunMotor 	:1;
unsigned Direction	:1;
unsigned unused		:14;
};
extern struct MotorFlags Flags;

extern unsigned int HallValue;
extern unsigned int timer3value;
extern unsigned int timer3avg;
extern unsigned char polecount;

extern unsigned int StateTableFwd[];
extern unsigned int StateTableRev[];

