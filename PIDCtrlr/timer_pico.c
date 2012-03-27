#include <timer_pico.h>

void reset_timer0(void)
{
    TCNT0 = timer0_ovrflow_cnt = 0;
}
void reset_timer1(void)
{
    TCNT1 = timer1_ovrflow_cnt = 0;
}
void init_timer0(const u08 prescaler)
{
  TCCR0B = prescaler;
  TIMSK0 = _BV(TOIE0); // enable interrupts
  reset_timer0(); // reset counter
}
void init_timer1(const u08 prescaler)
{
  TCCR1B = prescaler;
  TIMSK1 = _BV(TOIE1); // enable interrupts
  reset_timer1(); // reset counter
}
const uint32_t get_timer0_overflow(void)
{
    return timer0_ovrflow_cnt;
}
void delay_us(unsigned short time_us) 
{
	unsigned short delay_loops;
	register unsigned short i;

	delay_loops = ((time_us * CYCLES_PER_US)+3) / (2*5); // +3 for rounding up (dirty) 

	// one loop takes 5 cpu cycles 
	for (i=0; i < delay_loops; i++) {
		asm("nop");
	};
}

ISR(TIMER0_OVF_vect) 
{
  timer0_ovrflow_cnt++;
}
