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
*	This timerx8.h file has been heavily modified by SoR, but original
*	documentation has been left alone
*
****************************************************************************/
//*****************************************************************************
//
// File Name	: 'timerx8.h'
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
///	\ingroup driver_avr
/// \defgroup timerx8 Timer Function Library for ATmegaXX8 (timerx8.c)
/// \code #include "timerx8.h" \endcode
/// \par Overview
///		This library provides functions for use with the timers internal
///		to the AVR processors.  Functions include initialization, set prescaler,
///		calibrated pause function (in milliseconds), attaching and detaching of
///		user functions to interrupts, overflow counters, PWM. Arbitrary
///		frequency generation has been moved to the Pulse Library.
///
/// \par About Timers
///		The Atmel AVR-series processors each contain at least one
///		hardware timer/counter.  Many of the processors contain 2 or 3
///		timers.  Generally speaking, a timer is a hardware counter inside
///		the processor which counts at a rate related to the main CPU clock
///		frequency.  Because the counter value increasing (counting up) at
///		a precise rate, we can use it as a timer to create or measure 
///		precise delays, schedule events, or generate signals of a certain
///		frequency or pulse-width.
/// \par
///		As an example, the ATmega163 processor has 3 timer/counters.
///		Timer0, Timer1, and Timer2 are 8, 16, and 8 bits wide respectively.
///		This means that they overflow, or roll over back to zero, at a
///		count value of 256 for 8bits or 65536 for 16bits.  A prescaler is
///		avaiable for each timer, and the prescaler allows you to pre-divide
///		the main CPU clock rate down to a slower speed before feeding it to
///		the counting input of a timer.  For example, if the CPU clock
///		frequency is 3.69MHz, and Timer0's prescaler is set to divide-by-8,
///		then Timer0 will "tic" at 3690000/8 = 461250Hz.  Because Timer0 is
///		an 8bit timer, it will count to 256 in just 256/461250Hz = 0.555ms.
///		In fact, when it hits 255, it will overflow and start again at
///		zero.  In this case, Timer0 will overflow 461250/256 = 1801.76
///		times per second.
/// \par
///		Timer0 can be used a number of ways simultaneously.  First, the
///		value of the timer can be read by accessing the CPU register \c TCNT0.
///		We could, for example, figure out how long it takes to execute a
///		C command by recording the value of \c TCNT0 before and after
///		execution, then subtract (after-before) = time elapsed.  Or we can
///		enable the overflow interrupt which goes off every time T0
///		overflows and count out longer delays (multiple overflows), or
///		execute a special periodic function at every overflow.
/// \par
///		The other timers (Timer1 and Timer2) offer all the abilities of
///		Timer0 and many more features.  Both T1 and T2 can operate as
///		general-purpose timers, but T1 has special hardware allowing it to
///		generate PWM signals, while T2 is specially designed to help count
///		out real time (like hours, minutes, seconds).  See the
///		Timer/Counter section of the processor datasheet for more info.
///
//*****************************************************************************
//@{


#include "global.h"

// constants/macros/typdefs

#ifndef PWM10
	// mega128 PWM bits
	#define PWM10	WGM10
	#define PWM11	WGM11
#endif


#ifdef OCR0	// for processors that support output compare on Timer0


/*

/// Enter standard PWM Mode on timer0.
/// \param bitRes	indicates the period/resolution to use for PWM output in timer bits.
///						Must be either 8, 9, or 10 bits corresponding to PWM periods of 256, 512, or 1024 timer tics.
void timer0PWMInit(u08 bitRes);

/// Enter PWM Mode on timer0 with a specific top-count value.
/// \param topcount	indicates the desired PWM period in timer tics.
///						Can be a number between 1 and 65535 (16-bit).
void timer0PWMInitICR(u16 topcount);

/// Turn off all timer0 PWM output and set timer mode to normal.
void timer0PWMOff(void);

/// Turn on/off timer0 PWM outputs.
void timer0PWMAOn(void);			///< Turn on timer0 Channel A PWM output
void timer0PWMBOn(void);			///< Turn on timer0 Channel B PWM output
void timer0PWMCOn(void);			///< Turn on timer0 Channel C PWM output
void timer0PWMAOff(void);			///< turn off timer0 Channel A PWM output
void timer0PWMBOff(void);			///< turn off timer0 Channel B PWM output
void timer0PWMCOff(void);			///< turn off timer0 Channel C PWM output

void timer0PWMASet(u16 pwmDuty);	///< set duty of timer0 Channel A PWM output
void timer0PWMBSet(u16 pwmDuty);	///< set duty of timer0 Channel B PWM output
void timer0PWMCSet(u16 pwmDuty);	///< set duty of timer0 Channel C PWM output




/// Enter standard PWM Mode on timer1.
/// \param bitRes	indicates the period/resolution to use for PWM output in timer bits.
///						Must be either 8, 9, or 10 bits corresponding to PWM periods of 256, 512, or 1024 timer tics.
void timer1PWMInit(u08 bitRes);

/// Enter PWM Mode on timer1 with a specific top-count value.
/// \param topcount	indicates the desired PWM period in timer tics.
///						Can be a number between 1 and 65535 (16-bit).
void timer1PWMInitICR(u16 topcount);

/// Turn off all timer1 PWM output and set timer mode to normal.
void timer1PWMOff(void);

/// Turn on/off Timer1 PWM outputs.
void timer1PWMAOn(void);			///< Turn on timer1 Channel A PWM output
void timer1PWMBOn(void);			///< Turn on timer1 Channel B PWM output
void timer1PWMCOn(void);			///< Turn on timer1 Channel C PWM output
void timer1PWMAOff(void);			///< turn off timer1 Channel A PWM output
void timer1PWMBOff(void);			///< turn off timer1 Channel B PWM output
void timer1PWMCOff(void);			///< turn off timer1 Channel C PWM output

void timer1PWMASet(u16 pwmDuty);	///< set duty of timer1 Channel A PWM output
void timer1PWMBSet(u16 pwmDuty);	///< set duty of timer1 Channel B PWM output
void timer1PWMCSet(u16 pwmDuty);	///< set duty of timer1 Channel C PWM output

*/

/// Enter standard PWM Mode
/// \param bitRes	indicates the period/resolution to use for PWM output in timer bits.
///						Must be either 8, 9, or 10 bits corresponding to PWM periods of 256, 512, or 1024 timer tics.
void PWM_Init_timer1_LED(u08 bitRes);
void PWM_Init_timer2_H6(u08 bitRes);
void PWM_Init_timer3_E3(u08 bitRes);
void PWM_Init_timer3_E4(u08 bitRes);
void PWM_Init_timer3_E5(u08 bitRes);
void PWM_Init_timer4_H3(u08 bitRes);
void PWM_Init_timer4_H4(u08 bitRes);
void PWM_Init_timer4_H5(u08 bitRes);

/// Enter PWM Mode on timer2 with a specific top-count value.
/// \param topcount	indicates the desired PWM period in timer tics.
///						Can be a number between 1 and 65535 (16-bit).
void timer1PWMInitICR(u16 topcount);
void timer2PWMInitICR(u16 topcount);
void timer3PWMInitICR(u16 topcount);
void timer4PWMInitICR(u16 topcount);

/// Turn off all PWM output and set timer mode to normal.
void PWM_timer1_Off_All(void);
void PWM_timer2_Off_All(void);
void PWM_timer3_Off_All(void);
void PWM_timer4_Off_All(void);

/// Turn on PWM outputs.
void PWM_timer1_On_LED(void);
void PWM_timer2_On_H6(void);
void PWM_timer3_On_E3(void);
void PWM_timer3_On_E4(void);
void PWM_timer3_On_E5(void);
void PWM_timer4_On_H3(void);
void PWM_timer4_On_H4(void);
void PWM_timer4_On_H5(void);

/// Turn on PWM outputs.
void PWM_timer1_Off_LED(void);
void PWM_timer2_Off_H6(void);
void PWM_timer3_Off_E3(void);
void PWM_timer3_Off_E4(void);
void PWM_timer3_Off_E5(void);
void PWM_timer4_Off_H3(void);
void PWM_timer4_Off_H4(void);
void PWM_timer4_Off_H5(void);

///< set duty of timer2 PWM output
void PWM_timer1_Set_LED(u16 pwmDuty);
void PWM_timer2_Set_H6(u16 pwmDuty);
void PWM_timer3_Set_E3(u16 pwmDuty);
void PWM_timer3_Set_E4(u16 pwmDuty);
void PWM_timer3_Set_E5(u16 pwmDuty);
void PWM_timer4_Set_H3(u16 pwmDuty);
void PWM_timer4_Set_H4(u16 pwmDuty);
void PWM_timer4_Set_H5(u16 pwmDuty);	



#endif

