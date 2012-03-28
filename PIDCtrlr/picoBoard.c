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
#define WHEEL_RADIUS			5		// cm
#define RIGHT_WHEEL_TIMEOUT		0.15	// sec
#define LEFT_WHEEL_TIMEOUT		0.15	// sec
#define M_2PIR					31.4159	// Constant for convering from cycles per second to cm/sec

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

void wheel_r(float cmd_vel_r){
	u16 duty;
	// Limit velocity commands, cm/sec
	if(cmd_vel_r > 32)  cmd_vel_r =  32;
	if(cmd_vel_r < -32) cmd_vel_r = -32;
  
	duty = 1.1127*cmd_vel_r + 374.2424;
	
	PWM_timer1_Set_Pin10(duty);
}

void wheel_l(float cmd_vel_l){
	u16 duty;
	// Limit velocity commands, cm/sec
	if(cmd_vel_l > 32)  cmd_vel_l =  32;
	if(cmd_vel_l < -32) cmd_vel_l = -32;
  
	duty = -0.9697*cmd_vel_l + 369.6449;
	
	PWM_timer1_Set_Pin9(duty);
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

	u08 key;
	u08 counter = 1;

	float rps_r = 0;
	float rps_r_prev = 0;
	float rps_rf = 0;
	float rps_rf_prev = 0;
	float Kr = 25;
	float dt_r = 0;

	float rps_l = 0;
	float rps_l_prev = 0;
	float rps_lf = 0;
	float rps_lf_prev = 0;
	float Kl = 10;
	float dt_l = 0;

	float Kp_l = 1;
	float error_l;

	float v_l;
	float v_r;
	
	setup_hardware();
	//PWM_timer1_Set_Pin9(0);
	//PWM_timer1_Set_Pin10(0);
	
/**/
	while(1){
		elapsed_time_l = elapsed_time_r = ((get_timer0_overflow()*255 + TCNT0) * 0.0435) / 10000;
		
		l_count_current = get_left_count();
		r_count_current = get_right_count();

		
		

		if(l_count_current != l_count_previous){
			l_count_current = get_left_count();
			dt_l = (elapsed_time_l - elapsed_time_l_previous);		
			ticks_per_sec_l = (l_count_current - l_count_previous)/dt_l;

			rps_l = ticks_per_sec_l/(TICKS_PER_ROTATION);
			rps_lf = ((Kl*dt_l)/(Kl*dt_l + 2))*(rps_l + rps_l_prev) - ((Kl*dt_l - 2)/(Kl*dt_l + 2))*rps_lf_prev;

			rps_l_prev = rps_l;
			rps_lf_prev = rps_lf;

			//rprintf("Left: ");
			//rprintfFloat(5,rps_lf);
			//rprintf("\t\t,");
			//rprintfFloat(5,elapsed_time_l);
			//rprintfCRLF();
			
			l_count_previous = l_count_current;
			
			elapsed_time_l_previous = elapsed_time_l;
		}
		else if((elapsed_time_l - elapsed_time_l_previous) > LEFT_WHEEL_TIMEOUT){
			rps_lf = 0;
		}
		
		if(r_count_current != r_count_previous){
			r_count_current = get_right_count();
			dt_r = (elapsed_time_r - elapsed_time_r_previous);
			ticks_per_sec_r = (r_count_current - r_count_previous)/dt_r;
			
			rps_r = ticks_per_sec_r/(TICKS_PER_ROTATION);
			rps_rf = ((Kr*dt_r)/(Kr*dt_r + 2))*(rps_r + rps_r_prev) - ((Kr*dt_r - 2)/(Kr*dt_r + 2))*rps_rf_prev;

			rps_r_prev = rps_r;
			rps_rf_prev = rps_rf;

			//rprintf("\tRight: ");
			//rprintfFloat(5,rps_rf);
			//rprintf("\t\t,");
			//rprintfFloat(5,elapsed_time_r);
			//rprintfCRLF();
			
			
			r_count_previous = r_count_current;
			
			elapsed_time_r_previous = elapsed_time_r;
		}
		else if(((elapsed_time_r - elapsed_time_r_previous)) > RIGHT_WHEEL_TIMEOUT){
			rps_rf = 0;
		}
		
		v_l = M_2PIR*rps_lf;		
		v_r = M_2PIR*rps_rf;

		wheel_l(12);

		wheel_r(12);

		//rprintf("Left: ");
		rprintfFloat(5,v_l);
		//rprintf("Right: ");
		rprintf("\t,");
		rprintfFloat(5,v_r);
		rprintf("\t,");
		rprintfFloat(5,elapsed_time_l);
		rprintfCRLF();
		
		
	}

	return 0;

}


