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

//define port functions; example: PORT_ON( PORTD, 6);
#define PORT_ON( port_letter, number )			port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )			port_letter &= ~(1<<number)


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

	setup_hardware();
	PWM_timer1_Set_Pin9(200);
	PWM_timer1_Set_Pin10(200);
	
/**/
	while(1){
//		PORT_ON(PORTB,4);
//		delay_us(100);
//		PORT_OFF(PORTB,4);
//		delay_us(100);
//		if(bit_is_set(PIND,7)){
//			PORT_ON(PORTB,4);
//		}
//		else{
//			PORT_OFF(PORTB,4);
//		}
	}

	return 0;

}

SIGNAL(PCINT2_vect){
	// Only enabled for Digital Pin 7, Encoder Left
	if(bit_is_set(PIND,7)){
		PORT_ON(PORTB,4);
		//delay_us(10);
	}
	else{
		PORT_OFF(PORTB,4);
		//delay_us(10);
	}
	
}
SIGNAL(PCINT0_vect){
	// Enabled for Digital Pin 8, Encoder Left
	// and Digital Pin 12 and 13, Encoder Right
	if(bit_is_set(PINB,4)){
		PORT_ON(PORTB,0);
		//delay_us(10);
	}
	else{
		PORT_OFF(PORTB,0);
		//delay_us(10);
	}
	
}


ISR(BADISR_vect){
	//PORT_ON(PORTB,4);
}
