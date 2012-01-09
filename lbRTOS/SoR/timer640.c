#include "timer640.h"

#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include "global.h"
#include "rprintf.h"

#define TIMER_PRESCALE_MASK 0x07
#define RUN_USER_DEFINE_INTERRUPT(interrupt) if (TimerIntFunc[interrupt]) TimerIntFunc[interrupt]();


// Program ROM constants
// the prescale division values stored in order of timer control register index
// STOP, CLK, CLK/8, CLK/64, CLK/256, CLK/1024
uint16_t TimerPrescaleFactor[] PROGMEM = {0, 1, 8, 64, 256, 1024};
uint16_t Timer2PrescaleFactor[] PROGMEM = {0, 1, 8, 32, 64, 128, 256, 1024};

volatile uint32_t timer0_ovrflow_cnt;
volatile uint32_t timer1_ovrflow_cnt;
volatile uint32_t timer2_ovrflow_cnt;
volatile uint32_t timer3_ovrflow_cnt;
volatile uint32_t timer4_ovrflow_cnt;
volatile uint32_t timer5_ovrflow_cnt;
volatile uint32_t timer_sleep_cnt;

typedef void (*void_func_ptr)(void);
volatile static void_func_ptr TimerIntFunc[TIMER_MAX_ENUM];

uint16_t prescaler_hex_to_value(uint8_t hex)
{
  // basically pgm_read_word will return a pointer to TimerPrescaleFactor in program space
  // and it will move the array to the correct position, then typecast it back
  // to uint16_t
  return (uint16_t)(pgm_read_word(TimerPrescaleFactor+(hex & TIMER_PRESCALE_MASK)));
}

uint16_t prescaler_hex_to_value_for_timer2(uint8_t hex)
{
  // basically pgm_read_word will return a pointer to Timer2PrescaleFactor in program space
  // and it will move the array to the correct position, then typecast it back
  // to uint16_t
  return (uint16_t)(pgm_read_word(Timer2PrescaleFactor+(hex & TIMER_PRESCALE_MASK)));
}

uint16_t get_timer0_prescaler(void)
{
  return prescaler_hex_to_value(TCCR0B);
}
uint16_t get_timer1_prescaler(void)
{
  return prescaler_hex_to_value(TCCR1B);
}
uint16_t get_timer2_prescaler(void)
{
  return prescaler_hex_to_value_for_timer2(TCCR2B);
}
uint16_t get_timer3_prescaler(void)
{
  return prescaler_hex_to_value(TCCR3B);
}
uint16_t get_timer4_prescaler(void)
{
  return prescaler_hex_to_value(TCCR4B);
}
uint16_t get_timer5_prescaler(void)
{
  return prescaler_hex_to_value(TCCR5B);
}

const uint32_t get_timer0_overflow(void)
{
    return timer0_ovrflow_cnt;
}
const uint32_t get_timer1_overflow(void)
{
    return timer1_ovrflow_cnt;
}
const uint32_t get_timer2_overflow(void)
{
    return timer2_ovrflow_cnt;
}
const uint32_t get_timer3_overflow(void)
{
    return timer3_ovrflow_cnt;
}
const uint32_t get_timer4_overflow(void)
{
    return timer4_ovrflow_cnt;
}
const uint32_t get_timer5_overflow(void)
{
    return timer5_ovrflow_cnt;
}

const uint8_t get_timer0_counter(void)
{
    return TCNT0;
}
const uint16_t get_timer1_counter(void)
{
    return TCNT1;
}
const uint8_t get_timer2_counter(void)
{
    return TCNT2;
}
const uint16_t get_timer3_counter(void)
{
    return TCNT3;
}
const uint16_t get_timer4_counter(void)
{
    return TCNT4;
}
const uint16_t get_timer5_counter(void)
{
    return TCNT5;
}

void reset_timer0(void)
{
    TCNT0 = timer0_ovrflow_cnt = 0;
}
void reset_timer1(void)
{
    TCNT1 = timer1_ovrflow_cnt = 0;
}
void reset_timer2(void)
{
    TCNT2 = timer2_ovrflow_cnt = 0;
}
void reset_timer3(void)
{
    TCNT3 = timer3_ovrflow_cnt = 0;
}
void reset_timer4(void)
{
    TCNT4 = timer4_ovrflow_cnt = 0;
}
void reset_timer5(void)
{
    TCNT5 = timer5_ovrflow_cnt = 0;
}

void delay_us(unsigned short time_us) 
{
	unsigned short delay_loops;
	register unsigned short i;

	delay_loops = ((time_us * CYCLES_PER_US)+3) / 5; // +3 for rounding up (dirty) 

	// one loop takes 5 cpu cycles 
	for (i=0; i < delay_loops; i++) {};
}

void init_timer0(const uint8_t prescaler)
{
  TCCR0B = prescaler;
  TIMSK0 = _BV(TOIE0); // enable interrupts
  reset_timer0(); // reset counter
}
void init_timer1(const uint8_t prescaler)
{
  TCCR1B = prescaler;
  TIMSK1 = _BV(TOIE1); // enable interrupts
  reset_timer1(); // reset counter
}
void init_timer2(const uint8_t prescaler)
{
  TCCR2B = prescaler;
  TIMSK2 = _BV(TOIE2); // enable interrupts
  reset_timer2(); // reset counter
}
void init_timer3(const uint8_t prescaler)
{
  TCCR3B = prescaler;
  TIMSK3 = _BV(TOIE3); // enable interrupts
  reset_timer3(); // reset counter
}
void init_timer4(const uint8_t prescaler)
{
  TCCR4B = prescaler;
  TIMSK4 = _BV(TOIE4); // enable interrupts
  reset_timer4(); // reset counter
}
void init_timer5(const uint8_t prescaler)
{
  TCCR5B = prescaler;
  TIMSK5 = _BV(TOIE5); // enable interrupts
  reset_timer5(); // reset counter
}

void timer_attach(TimerInterrupt_t interrupt, void (*user_func)(void) )
{
  // set the interrupt function to run
  // the supplied user's function
  TimerIntFunc[interrupt] = user_func;
}

void timer_detach(TimerInterrupt_t interrupt)
{
  // clear the user defined interrupt function
  TimerIntFunc[interrupt] = NULL;
}

void
_delay_loop_2(uint16_t __count)
{
	__asm__ volatile (
		"1: sbiw %0,1" "\n\t"
		"brne 1b"
		: "=w" (__count)
		: "0" (__count)
	);
}

void sleep(uint16_t time_ms)
{
	uint16_t __ticks;
	double __tmp = ((F_CPU) / 4e3) * time_ms;
	if (__tmp < 1.0)
		__ticks = 1;
	else if (__tmp > 65535)
	{
		//	__ticks = requested delay in 1/10 ms
		__ticks = (uint16_t) (time_ms * 10.0);
		while(__ticks)
		{
			// wait 1/10 ms
			_delay_loop_2(((F_CPU) / 4e3) / 10);
			__ticks --;
		}
		return;
	}
	else
		__ticks = (uint16_t)__tmp;
	_delay_loop_2(__ticks);
}
/*{
  // pauses for exactly <time_ms> number of milliseconds
  uint8_t timerThres;
  uint32_t ticRateHz;
  uint32_t pause;

  // capture current pause timer value
  timerThres = TCNT0;
  // reset pause timer overflow count
  timer_sleep_cnt = 0;
  // calculate delay for [pause_ms] milliseconds
  // prescaler division = 1<<(pgm_read_byte(TimerPrescaleFactor+inb(TCCR0)))
  ticRateHz = F_CPU/get_timer0_prescaler();
  // precision management
  // prevent overflow and precision underflow
  //	-could add more conditions to improve accuracy
  if( ((ticRateHz < 429497) && (time_ms <= 10000)) )
    pause = (time_ms*ticRateHz)/1000;
  else
    pause = time_ms*(ticRateHz/1000);

  // loop until time expires
  while( ((timer_sleep_cnt<<8) | (TCNT0)) < (pause+timerThres) )
  {
    if( timer_sleep_cnt < (pause>>8));
    {
//      rprintf("putting axon to sleep...\n");
      // save power by idling the processor
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();
//      rprintf("done with sleep_mode()...\n");
    }
  }
}*/

ISR(TIMER0_OVF_vect) 
{
  timer0_ovrflow_cnt++;
  timer_sleep_cnt++;
  RUN_USER_DEFINE_INTERRUPT(TIMER0_OVF_interrupt);
}
ISR(TIMER1_OVF_vect) 
{
  timer1_ovrflow_cnt++;
  RUN_USER_DEFINE_INTERRUPT(TIMER1_OVF_interrupt);
}
ISR(TIMER2_OVF_vect) 
{
  timer2_ovrflow_cnt++;
  RUN_USER_DEFINE_INTERRUPT(TIMER2_OVF_interrupt);
}
ISR(TIMER3_OVF_vect) 
{
  timer3_ovrflow_cnt++;
  RUN_USER_DEFINE_INTERRUPT(TIMER3_OVF_interrupt);
}
ISR(TIMER4_OVF_vect) 
{
  timer4_ovrflow_cnt++;
  RUN_USER_DEFINE_INTERRUPT(TIMER4_OVF_interrupt);
}
ISR(TIMER5_OVF_vect) 
{
  timer5_ovrflow_cnt++;
  RUN_USER_DEFINE_INTERRUPT(TIMER5_OVF_interrupt);
}
ISR(TIMER0_COMPA_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER0_COMPA_interrupt);
}
ISR(TIMER0_COMPB_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER0_COMPB_interrupt);
}
ISR(TIMER1_CAPT_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER1_CAPT_interrupt);
}
//This interrupt service routine is being used by and has
//been previously defined by FreeRTOS port.c line 437
//for the tick interrupt.
//Uncommenting this line will cause the code not to
//compile.
//01/10/11 Griswald Brooks
/*ISR(TIMER1_COMPA_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER1_COMPA_interrupt);
}
*/
ISR(TIMER1_COMPB_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER1_COMPB_interrupt);
}
ISR(TIMER1_COMPC_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER1_COMPC_interrupt);
}
ISR(TIMER2_COMPA_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER2_COMPA_interrupt);
}
ISR(TIMER2_COMPB_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER2_COMPB_interrupt);
}
ISR(TIMER3_CAPT_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER3_CAPT_interrupt);
}
ISR(TIMER3_COMPA_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER3_COMPA_interrupt);
}
ISR(TIMER3_COMPB_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER3_COMPB_interrupt);
}
ISR(TIMER3_COMPC_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER3_COMPC_interrupt);
}
ISR(TIMER4_CAPT_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER4_CAPT_interrupt);
}
ISR(TIMER4_COMPA_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER4_COMPA_interrupt);
}
ISR(TIMER4_COMPB_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER4_COMPB_interrupt);
}
ISR(TIMER4_COMPC_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER4_COMPC_interrupt);
}
ISR(TIMER5_CAPT_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER5_CAPT_interrupt);
}
ISR(TIMER5_COMPA_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER5_COMPA_interrupt);
}
ISR(TIMER5_COMPB_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER5_COMPB_interrupt);
}
ISR(TIMER5_COMPC_vect)
{
  RUN_USER_DEFINE_INTERRUPT(TIMER5_COMPC_interrupt);
}

ISR(BADISR_vect)
{
  rprintf("BAD_vect called!");
}
