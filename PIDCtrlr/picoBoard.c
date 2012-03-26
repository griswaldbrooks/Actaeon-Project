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

//define port functions; example: PORT_ON( PORTD, 6);
#define PORT_ON( port_letter, number )			port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )			port_letter &= ~(1<<number)

void pwm_setup(void){

	PWM_timer1_On_Pin9();
	PWM_timer1_On_Pin10();

	timer1PWMInitICR(5000);

	// Set pins B1 and B2 to output
	sbi(DDRB,DDB1);		// Pin 9
	sbi(DDRB,DDB2);		// Pin 10
}

void setup_hardware(void){
	// Set pins PD0 and PD1 for UART RX and TX respectively
	DDRD = (0<<DDD0)|(1<<DDD1);
	init_timer0(TIMER_CLK_64);
	init_timer1(TIMER_CLK_64);
	pwm_setup();
}



int main(void){

	setup_hardware();
	PWM_timer1_Set_Pin9(200);
	PWM_timer1_Set_Pin10(200);
	
	/*
	while(1){
	//for(u16 iter = 0; iter < 2000; iter++){
		PORT_ON(PORTB,0);
		delay_us(100);
		PORT_OFF(PORTB,0);
		delay_us(100);
	}
	*/
	return 0;

}
