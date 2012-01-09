/****************************************************************************
*
*   Copyright (c) 2008 www.societyofrobots.com
*   (please link back if you use this code!)
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*	This timerx8.c file has been heavily modified by SoR, but original
*	documentation has been left alone
*
****************************************************************************/
//*****************************************************************************
//
// File Name	: 'timerx8.c'
// Title		: Timer function library for ATmegaXX8 Processors
// Author		: Pascal Stang - Copyright (C) 2000-2005
// Created		: 11/22/2000
// Revised		: 06/15/2005
// Version		: 1.0
// Target MCU	: Atmel AVR Series
// Editor Tabs	: 4
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

/*
Two 8-bit Timer/Counters with Separate Prescaler and Compare Mode
Four 16-bit Timer/Counter with Separate Prescaler, Compare- and Capture Mode
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "global.h"
#include "pwm.h"
#include "rprintf.h"



/*
ATmega640: Four 8-bit PWM Channels, Six/Twelve PWM
Channels with Programmable Resolution from 2 to 16 Bits


PWM pins on Axon:
OC0A  B7 (not connected)
OC0B  G5 (attached to button)
OC1A  B5 (not connected)
OC1B  B6 (attached to green LED)
OC1C  B7 (not connected)
OC2A  B4 (not connected)
OC2B  H6
OC3A  E3
OC3B  E4
OC3C  E5
OC4A  H3
OC4B  H4
OC4C  H5
OC5A  L3 (not connected)
OC5B  L4 (not connected)
OC5C  L5 (not connected)

explainations/examples:
http://www.societyofrobots.com/robotforum/index.php?topic=1827.0
http://www.societyofrobots.com/robotforum/index.php?topic=5590.0

If you use this, then you cannot use the associated timers for other things.
H6 	 uses timer2
E3-5 uses timer3
H3-5 uses timer4
Unfortunately, the 3 pins that use timer5 are not connected on the Axon board, so they are
not available for PWM. The upside is that timer5 is available for other use without interference.

OCR is the PWM on from 0, while ICR is the total PWM length

|----|___________________
  OCR
  		  ICR

to calculate PWM: (desired ICR time, seconds)*(clock frequency, 1/seconds)/prescaler = TOP
TOP=TOP/2 if using phase and frequency correct mode

adjust the duty cycle of the output pin by setting OCR1A to a value between 0 and ICR1

PWM tutorial: http://www.societyofrobots.com/member_tutorials/node/228
*/

//tested and working: H3, H6

//OC1B  pin B6 (attached to green LED)
void PWM_Init_timer1_LED(u08 bitRes)
{
	// enable timer2 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR1A,PWM11);
		cbi(TCCR1A,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR1A,PWM11);
		sbi(TCCR1A,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR1A,PWM11);
		sbi(TCCR1A,PWM10);
	}
	// clear output compare values
	OCR1B = 0;
}
//pin H6, timer2
void PWM_Init_timer2_H6(u08 bitRes)
{
	// enable timer2 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR2A,PWM11);
		cbi(TCCR2A,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR2A,PWM11);
		sbi(TCCR2A,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR2A,PWM11);
		sbi(TCCR2A,PWM10);
	}
	// clear output compare values
	OCR2B = 0;
}
//pin E3
void PWM_Init_timer3_E3(u08 bitRes)
{
	// enable timer3 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR3A,PWM11);
		cbi(TCCR3A,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR3A,PWM11);
		sbi(TCCR3A,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR3A,PWM11);
		sbi(TCCR3A,PWM10);
	}
	// clear output compare values
	OCR3A = 0;
	//timer3PWMInitICR(20000);// 20mS PWM cycle time for RC servos
}
//pin E4
void PWM_Init_timer3_E4(u08 bitRes)
{
	// enable timer3 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR3B,PWM11);
		cbi(TCCR3B,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR3B,PWM11);
		sbi(TCCR3B,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR3B,PWM11);
		sbi(TCCR3B,PWM10);
	}
	// clear output compare values
	OCR3B = 0;
	//timer3PWMInitICR(20000);// 20mS PWM cycle time for RC servos
}
//pin E5
void PWM_Init_timer3_E5(u08 bitRes)
{
	// enable timer3 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR3C,PWM11);
		cbi(TCCR3C,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR3C,PWM11);
		sbi(TCCR3C,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR3C,PWM11);
		sbi(TCCR3C,PWM10);
	}
	// clear output compare values
	OCR3C = 0;
	//timer3PWMInitICR(20000);// 20mS PWM cycle time for RC servos
}
//pin H3, timer4
void PWM_Init_timer4_H3(u08 bitRes)
{
	// enable timer4 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR4A,PWM11);
		cbi(TCCR4A,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR4A,PWM11);
		sbi(TCCR4A,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR4A,PWM11);
		sbi(TCCR4A,PWM10);
	}
	// clear output compare values
	OCR4A = 0;
	//timer4PWMInitICR(20000);// 20mS PWM cycle time for RC servos
}
//pin H4, timer4
void PWM_Init_timer4_H4(u08 bitRes)
{
	// enable timer4 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR4B,PWM11);
		cbi(TCCR4B,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR4B,PWM11);
		sbi(TCCR4B,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR4B,PWM11);
		sbi(TCCR4B,PWM10);
	}
	// clear output compare values
	OCR4B = 0;
	//timer4PWMInitICR(20000);// 20mS PWM cycle time for RC servos
}
//pin H5, timer4
void PWM_Init_timer4_H5(u08 bitRes)
{
	// enable timer4 as 8,9,10bit PWM
	if(bitRes == 9)
	{	// 9bit mode
		sbi(TCCR4A,PWM11);
		cbi(TCCR4A,PWM10);
	}
	else if( bitRes == 10 )
	{	// 10bit mode
		sbi(TCCR4A,PWM11);
		sbi(TCCR4A,PWM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR4A,PWM11);
		sbi(TCCR4A,PWM10);
	}
	// clear output compare values
	OCR4C = 0;
	//timer4PWMInitICR(20000);// 20mS PWM cycle time for RC servos
}

#ifdef WGM10
// include support for arbitrary top-count PWM
// on new AVR processors that support it
void timer1PWMInitICR(u16 topcount)
{
	// set PWM mode with ICR top-count
	cbi(TCCR1A,WGM10);
	sbi(TCCR1A,WGM11);
	sbi(TCCR1B,WGM12);
	sbi(TCCR1B,WGM13);
	
	// set top count value
	ICR1 = topcount;
	
	// clear output compare values
	OCR1A = 0;
	OCR1B = 0;
	OCR1C = 0;
}
void timer3PWMInitICR(u16 topcount)
{
	// set PWM mode with ICR top-count
	cbi(TCCR3A,WGM10);
	sbi(TCCR3A,WGM11);
	sbi(TCCR3B,WGM12);
	sbi(TCCR3B,WGM13);
	
	// set top count value
	ICR3 = topcount;
	
	// clear output compare values
	OCR3A = 0;
	OCR3B = 0;
	OCR3C = 0;
}
void timer4PWMInitICR(u16 topcount)
{
	// set PWM mode with ICR top-count
	cbi(TCCR4A,WGM10);
	sbi(TCCR4A,WGM11);
	sbi(TCCR4B,WGM12);
	sbi(TCCR4B,WGM13);
	
	// set top count value
	ICR4 = topcount;
	
	// clear output compare values
	OCR4A = 0;
	OCR4B = 0;
	OCR4C = 0;
}
#endif

//on commands
void PWM_timer1_On_LED(void)
{
	sbi(TCCR1A,COM1B1);
	cbi(TCCR1A,COM1B0);
}
void PWM_timer2_On_H6(void)
{
	sbi(TCCR2A,COM2B1);
	cbi(TCCR2A,COM2B0);
}
void PWM_timer3_On_E3(void)
{
	sbi(TCCR3A,COM3A1);
	cbi(TCCR3A,COM3A0);
}
void PWM_timer3_On_E4(void)
{
	sbi(TCCR3A,COM3B1);
	cbi(TCCR3A,COM3B0);
}
void PWM_timer3_On_E5(void)
{
	sbi(TCCR3A,COM3C1);
	cbi(TCCR3A,COM3C0);
}
void PWM_timer4_On_H3(void)
{
	sbi(TCCR4A,COM4A1);
	cbi(TCCR4A,COM4A0);
}
void PWM_timer4_On_H4(void)
{
	sbi(TCCR4A,COM4B1);
	cbi(TCCR4A,COM4B0);
}
void PWM_timer4_On_H5(void)
{
	sbi(TCCR4A,COM4C1);
	cbi(TCCR4A,COM4C0);
}

//off commands
void PWM_timer1_Off_LED(void)
{
	cbi(TCCR1A,COM1B1);
	cbi(TCCR1A,COM1B0);
}
void PWM_timer2_Off_H6(void)
{
	cbi(TCCR2A,COM2B1);
	cbi(TCCR2A,COM2B0);
}
void PWM_timer3_Off_E3(void)
{
	cbi(TCCR3A,COM3A1);
	cbi(TCCR3A,COM3A0);
}
void PWM_timer3_Off_E4(void)
{
	cbi(TCCR3A,COM3B1);
	cbi(TCCR3A,COM3B0);
}
void PWM_timer3_Off_E5(void)
{
	cbi(TCCR3A,COM3C1);
	cbi(TCCR3A,COM3C0);
}
void PWM_timer4_Off_H3(void)
{
	cbi(TCCR4A,COM4A1);
	cbi(TCCR4A,COM4A0);
}
void PWM_timer4_Off_H4(void)
{
	cbi(TCCR4A,COM4B1);
	cbi(TCCR4A,COM4B0);
}
void PWM_timer4_Off_H5(void)
{
	cbi(TCCR4A,COM4C1);
	cbi(TCCR4A,COM4C0);
}


void PWM_timer1_Off_All(void)
{
	cbi(TCCR1A,PWM11);
	cbi(TCCR1A,PWM10);
	//timer2PWMAOff();
	PWM_timer1_Off_LED();
	//timer2PWMCOff();
}
void PWM_timer2_Off_All(void)
{
	cbi(TCCR2A,PWM11);
	cbi(TCCR2A,PWM10);
	//timer2PWMAOff();
	PWM_timer2_Off_H6();
	//timer2PWMCOff();
}
void PWM_timer3_Off_All(void)
{
	cbi(TCCR3A,PWM11);
	cbi(TCCR3A,PWM10);
	//timer2PWMAOff();
	PWM_timer3_Off_E3();
	PWM_timer3_Off_E4();
	PWM_timer3_Off_E5();
	//timer2PWMCOff();
}
void PWM_timer4_Off_All(void)
{
	cbi(TCCR4A,PWM11);
	cbi(TCCR4A,PWM10);
	//timer2PWMAOff();
	PWM_timer4_Off_H3();
	PWM_timer4_Off_H4();
	PWM_timer4_Off_H5();
	//timer2PWMCOff();
}


// set PWM (output compare) duty for channel B
// this PWM output is generated on OC2B pin
// NOTE:	pwmDuty should be in the range 0-255 for 8bit PWM
//			pwmDuty should be in the range 0-511 for 9bit PWM
//			pwmDuty should be in the range 0-1023 for 10bit PWM
void PWM_timer1_Set_LED(u16 pwmDuty)
	{OCR1B = pwmDuty;}
void PWM_timer2_Set_H6(u16 pwmDuty)
	{OCR2B = pwmDuty;}
void PWM_timer3_Set_E3(u16 pwmDuty)
	{OCR3A = pwmDuty;}
void PWM_timer3_Set_E4(u16 pwmDuty)
	{OCR3B = pwmDuty;}
void PWM_timer3_Set_E5(u16 pwmDuty)
	{OCR3C = pwmDuty;}
void PWM_timer4_Set_H3(u16 pwmDuty)
	{OCR4A = pwmDuty;}
void PWM_timer4_Set_H4(u16 pwmDuty)
	{OCR4B = pwmDuty;}
void PWM_timer4_Set_H5(u16 pwmDuty)
	{OCR4C = pwmDuty;}

