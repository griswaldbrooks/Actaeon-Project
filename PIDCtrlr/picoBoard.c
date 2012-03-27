//AVR includes
#include <avr/io.h>		    // include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support
#include <stdio.h>			// stuff
#include <stdlib.h>			// stuff
#include <math.h>			// stuff
//#include "libm.a"			// required with math.h
#include <string.h>			// allow strings to be used
#include <avr/eeprom.h>		// adds EEPROM functionality

#include <timer_pico.h>
#include <pwm_pico.h>
#include <uart4.h>
#include <rprintf.h>
#include <encoder_pico.h>

//define port functions; example: PORT_ON( PORTD, 6);
#define PORT_ON( port_letter, number )			port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )			port_letter &= ~(1<<number)

#define TICKS_PER_ROTATION		128

void pwm_setup(void){

	PWM_timer1_On_Pin9();
	PWM_timer1_On_Pin10();

	// 20 ms period
	timer1PWMInitICR(5000);

	// Set pins B1 and B2 to output for PWM
	sbi(DDRB,DDB1);		// Pin 9
	sbi(DDRB,DDB2);		// Pin 10
}

void setup_hardware(void){
	// Set pins PD0 and PD1 for UART RX and TX respectively
	cbi(DDRD,DDD0);		// Pin 0 RX
	sbi(DDRD,DDD1);		// Pin 1 TX
	uartInit();
	uartSetBaudRate(0,115200);
	rprintfInit(uart0SendByte);

	// Clear pins for encoders to input
	cbi(DDRD,DDD7);		// Pin 7  Left  Quadrature Encoder
	cbi(DDRB,DDB0);		// Pin 8  Left  Quadrature Encoder
	cbi(DDRB,DDB4);		// Pin 12 Right Quadrature Encoder
	cbi(DDRB,DDB5);		// Pin 13 Right Quadrature Encoder
	PORT_ON(PORTD,7);	// Enable pull-up resistor
	PORT_ON(PORTB,0);	// Enable pull-up resistor
	PORT_ON(PORTB,4);	// Enable pull-up resistor
	PORT_ON(PORTB,5);	// Enable pull-up resistor
	

	init_timer0(TIMER_CLK_64);
	init_timer1(TIMER_CLK_64);
	pwm_setup();

	// Enable DIO Pin 7 to interrupt on change
	sbi(PCICR,PCIE2);
	sbi(PCMSK2,PCINT23);// Pin 7
	
	sbi(PCICR,PCIE0);
	sbi(PCMSK0,PCINT0);	// Pin 8
	sbi(PCMSK0,PCINT4);	// Pin 12
	sbi(PCMSK0,PCINT5); // Pin 13


	// Enable interrupts
	sei();	// Global interrupt enable

}



int main(void){
	float elapsed_time_l = 0;
	float elapsed_time_r = 0;
	float elapsed_time_l_previous = 0;
	float elapsed_time_r_previous = 0;
	float ticks_per_sec_l = 0;
	float ticks_per_sec_r = 0;
	s32 l_count_current = 0;
	s32 l_count_previous = 0;
	s32 r_count_current = 0;
	s32 r_count_previous = 0;
	
	setup_hardware();
	PWM_timer1_Set_Pin9(500);
	PWM_timer1_Set_Pin10(500);
	
/**/
	while(1){
		elapsed_time_l = elapsed_time_r = ((get_timer0_overflow()*255 + TCNT0) * 0.0435) / 10000;
		
		l_count_current = get_left_count();
		r_count_current = get_right_count();
		
		
		if(l_count_current != l_count_previous){
			l_count_current = get_left_count();
			ticks_per_sec_l = (l_count_current - l_count_previous)/(elapsed_time_l - elapsed_time_l_previous);
			
			//rprintf("Left: ");
			//rprintfFloat(5,ticks_per_sec_l/(TICKS_PER_ROTATION));
			//rprintfCRLF();
			
			l_count_previous = l_count_current;
			
			elapsed_time_l_previous = elapsed_time_l;
		}
		
		if(r_count_current != r_count_previous){
			r_count_current = get_right_count();
			ticks_per_sec_r = (r_count_current - r_count_previous)/(elapsed_time_r - elapsed_time_r_previous);
			
			//rprintf("\tRight: ");
			rprintfFloat(5,ticks_per_sec_r/(TICKS_PER_ROTATION));
			rprintfCRLF();
			
			r_count_previous = r_count_current;
			
			elapsed_time_r_previous = elapsed_time_r;
		}

	}

	return 0;

}


