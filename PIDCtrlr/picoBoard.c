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
//#include <rprintf.h>
#include <encoder_pico.h>

//define port functions; example: PORT_ON( PORTD, 6);
#define PORT_ON( port_letter, number )			port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )			port_letter &= ~(1<<number)
#define wheel_l_on()							PWM_timer1_On_Pin9();
#define wheel_r_on()							PWM_timer1_On_Pin10();

#define ROBOT_DIAMETER			16.8	// cm
#define ROBOT_RADIUS			8.4		// cm
#define TICKS_PER_ROTATION		128
#define WHEEL_RADIUS			5		// cm
#define RIGHT_WHEEL_TIMEOUT		0.15	// sec
#define LEFT_WHEEL_TIMEOUT		0.15	// sec
#define M_2PIR					31.4159	// Constant for convering from cycles per second to cm/sec

// Global variables ///////////////////

// Upper Brain Receive Uart ///////////
#define UB_BUFFER_SIZE	8
uint8_t ub_buffer[UB_BUFFER_SIZE];
uint8_t *ub_buffer_write_ndx = NULL;
uint8_t *ub_buffer_read_ndx = NULL;
///////////////////////////////////////

// Encoding variables /////////////////
float elapsed_time_l = 0;
float elapsed_time_r = 0;
float elapsed_time_l_previous = 0;
float elapsed_time_r_previous = 0;
float elapsed_time;
float ticks_per_sec_l = 0;
float ticks_per_sec_r = 0;
s32 l_count_current = 0;
s32 l_count_previous = 0;
s32 r_count_current = 0;
s32 r_count_previous = 0;
///////////////////////////////////////
	
// Filter values for velocity encoding
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
////////////////////////////////////////

// PID constants //////////////////////////////////////////////////////////////
float error_l_n1, error_l;							// Error, previous
float error_r_n1, error_r;							// Error, previous

float u_l, up_l, ui_l, ud_l;						// Output, PID
float u_r, up_r, ui_r, ud_r;						// Output, PID

float ui_l_n1 = 0;									// Previous integral
float ui_r_n1 = 0;									// Previous integral

float ud_l_n1, udf_l, udf_l_n1;						// Previous derivative, filtered derivative value and previous filtered derivative value
float ud_r_n1, udf_r, udf_r_n1;						// Previous derivative, filtered derivative value and previous filtered derivative value
	
float Kp_l, Ki_l, Kd_l, K_l, T_l, Ti_l, Td_l;		// PID Constants
float Kp_r, Ki_r, Kd_r, K_r, T_r, Ti_r, Td_r;		// PID Constants
	
float lpf1_l, lpf2_l, lpf3_l;						// Low pass filter constants
float lpf1_r, lpf2_r, lpf3_r;						// Low pass filter constants

float wb = 1256.6;									// Break frequency in radians/sec
////////////////////////////////////////
		
// Wheel Linear Velcities //////////////
float v_l;
float v_r;
////////////////////////////////////////

// Robot Velocities ////////////////////
float ang_vel_robot;
float lin_vel_robot;
////////////////////////////////////////

// Robot Heading ///////////////////////
float heading_robot;
////////////////////////////////////////

// Robot Commands //////////////////////
float v_l_cmd;
float v_r_cmd;
float cmd_lin_vel = 0;
float cmd_ang_vel = 0;
////////////////////////////////////////

// Float to byte unions ////////////////
union u_vel{
	float f_vel;
	uint8_t arr_vel[4];
};

union u_ome{
	float f_ome;
	uint8_t arr_ome[4];
};
////////////////////////////////////////

void pwm_setup(void){

	// 20 ms period
	timer1PWMInitICR(5000);

	// Set pins B1 and B2 to output for PWM
	sbi(DDRB,DDB1);		// Pin 9
	sbi(DDRB,DDB2);		// Pin 10
}

void ubRcv(unsigned char c){
	if(c != 0xff){
		*ub_buffer_write_ndx = c;
		ub_buffer_write_ndx++;		
		if(ub_buffer_write_ndx >= (ub_buffer + UB_BUFFER_SIZE)){ 
			ub_buffer_write_ndx = ub_buffer;
		}
	}
}

uint8_t read_ub(){
	while(ub_buffer_read_ndx == ub_buffer_write_ndx){ delay_us(10); };
	uint8_t data = *ub_buffer_read_ndx;
	ub_buffer_read_ndx++;
	if(ub_buffer_read_ndx >= (ub_buffer + UB_BUFFER_SIZE)){ 
		ub_buffer_read_ndx = ub_buffer;
	}
	return data;
}

void init_ub_buffer(){
	ub_buffer_write_ndx = ub_buffer_read_ndx = ub_buffer;
}

void update_vw_cmd(){
	union u_vel fltuint8_velocity;
	union u_ome fltuint8_omega;
	uint8_t rcv_byte;
	uint8_t rcv_chk;
	uint8_t calc_chk;
	
	rcv_byte = read_ub();
	
	if(rcv_byte == 0xFA){
		
		fltuint8_velocity.arr_vel[0] = read_ub();
		fltuint8_velocity.arr_vel[1] = read_ub();
		fltuint8_velocity.arr_vel[2] = read_ub();
		fltuint8_velocity.arr_vel[3] = read_ub();	

		fltuint8_omega.arr_ome[0] = read_ub();
		fltuint8_omega.arr_ome[1] = read_ub();
		fltuint8_omega.arr_ome[2] = read_ub();
		fltuint8_omega.arr_ome[3] = read_ub();
		

		rcv_chk = read_ub();
		

		calc_chk = fltuint8_velocity.arr_vel[0] + fltuint8_velocity.arr_vel[1] + fltuint8_velocity.arr_vel[2]
			+ fltuint8_velocity.arr_vel[3] + fltuint8_omega.arr_ome[0] + fltuint8_omega.arr_ome[1] 
			+ fltuint8_omega.arr_ome[2] + fltuint8_omega.arr_ome[3];
/*
		rprintfu08(fltuint8_velocity.arr_vel[0]);
		rprintfu08(fltuint8_velocity.arr_vel[1]);
		rprintfu08(fltuint8_velocity.arr_vel[2]);
		rprintfu08(fltuint8_velocity.arr_vel[3]);

		rprintfu08(fltuint8_omega.arr_ome[0]);
		rprintfu08(fltuint8_omega.arr_ome[1]);
		rprintfu08(fltuint8_omega.arr_ome[2]);
		rprintfu08(fltuint8_omega.arr_ome[3]);
*/
		if(calc_chk == rcv_chk){
			
				cmd_lin_vel = fltuint8_velocity.f_vel;
				cmd_ang_vel = fltuint8_omega.f_ome;
				/*
				PORT_ON(PORTD,5);
				delay_us(200);
				PORT_OFF(PORTD,5);
				*/
				//rprintfCRLF();
				//rprintf("Received Velocity: ");
				//rprintfFloat(5,fltuint8_velocity.f_vel);
				//rprintf(" Omega: ");
				//rprintfFloat(5,fltuint8_omega.f_ome);
				//rprintfCRLF();
		}
			
	}
}

void setup_hardware(void){
	// Set pins PD0 and PD1 for UART RX and TX respectively
	cbi(DDRD,DDD0);		// Pin 0 RX
	sbi(DDRD,DDD1);		// Pin 1 TX
	uartInit();
	uartSetBaudRate(0,38400);
	rprintfInit(uart0SendByte);
	//UART ISR *** UART ISR ***
	init_ub_buffer();
	uartSetRxHandler(0, &ubRcv);
	//UART ISR *** UART ISR ***

	// Clear pins for encoders to input
	cbi(DDRD,DDD7);		// Pin 7  Left  Quadrature Encoder
	cbi(DDRB,DDB0);		// Pin 8  Left  Quadrature Encoder
	cbi(DDRB,DDB4);		// Pin 12 Right Quadrature Encoder
	cbi(DDRB,DDB5);		// Pin 13 Right Quadrature Encoder
	PORT_ON(PORTD,7);	// Enable pull-up resistor
	PORT_ON(PORTB,0);	// Enable pull-up resistor
	PORT_ON(PORTB,4);	// Enable pull-up resistor
	PORT_ON(PORTB,5);	// Enable pull-up resistor
	
	// Set test pin
	sbi(DDRD,DDD5);		// Pin 5  Digital Out
	PORT_OFF(PORTD,5);	// Set pin off

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
	// Wheel deadband 8cm/sec > v_dead > -5cm/sec
	if((cmd_vel_r < 8) && (cmd_vel_r > -8))  cmd_vel_r = 0;

	// Limit velocity commands, cm/sec
	if(cmd_vel_r > 32)  cmd_vel_r =  32;
	if(cmd_vel_r < -32) cmd_vel_r = -32;
  
	duty = 1.1127*cmd_vel_r + 374.2424;
	
	PWM_timer1_Set_Pin10(duty);
}

void wheel_l(float cmd_vel_l){
	u16 duty;
	// Wheel deadband 7cm/sec > v_dead > -9cm/sec
	if((cmd_vel_l < 9) && (cmd_vel_l > -9))  cmd_vel_l =  0;

	// Limit velocity commands, cm/sec
	if(cmd_vel_l > 32)  cmd_vel_l =  32;
	if(cmd_vel_l < -32) cmd_vel_l = -32;
  
	duty = -0.9697*cmd_vel_l + 369.6449;
	
	PWM_timer1_Set_Pin9(duty);
}


int main(void){
	
	// Initialize variables ///////////////////////////////////////////////////////////
	error_l_n1 = error_l = u_l = 0;
	error_r_n1 = error_r = u_r = 0;

	ud_l_n1 = udf_l = udf_l_n1 = 0;
	ud_r_n1 = udf_r = udf_r_n1 = 0;
	
	//K_l = 0.0008;
	K_l = 0.6;
	K_r = 0.6;
	//K_r = 0.0006;

	Ti_l = 1.1945;
	Td_l = 0.2986;
	T_l = 2.3890;										// sec

	Ti_r = 0.1100;
	Td_r = 0.0275;
	T_r = 0.2200;										// sec
		
	Kp_l = K_l;											// Proportional constant
	Ki_l = (K_l*T_l)/(2*Ti_l);							// Integral constant
	Kd_l = (2*K_l*Td_l)/T_l;							// Derivative constant

	Kp_r = 0.6*K_r;										// Proportional constant
	Ki_r = (K_r*T_r)/(2*Ti_r);							// Integral constant
	Kd_r = (2*K_r*Td_r)/T_r;							// Derivative constant
	
	lpf1_l = (wb*T_l)/(2 + wb*T_l);
	lpf2_l = lpf1_l;
	lpf3_l = (wb*T_l - 2)/(wb*T_l + 2);

	lpf1_r = (wb*T_r)/(2 + wb*T_r);
	lpf2_r = lpf1_r;
	lpf3_r = (wb*T_r - 2)/(wb*T_r + 2);
	///////////////////////////////////////////////////////////////////////////////////////
	
	setup_hardware();
	
	
/**/
	wheel_l(0);
	wheel_r(0);
	wheel_l_on();
	wheel_r_on();
	
	// Wait two seconds
	for(u08 iter = 0; iter < 5; iter++){
		for(u16 ndx = 0; ndx < 1250; ndx++){
			PORT_ON(PORTD,5);	// Set pin off
			delay_us(200);
		}
		for(u16 ndx = 0; ndx < 1250; ndx++){
			PORT_OFF(PORTD,5);	// Set pin off
			delay_us(200);
		}
	}
	//cmd_lin_vel = 15;
	while(1){
		
	// Velocity Encoding //////////////////////////////////////////////////////////////////////
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

			PORT_ON(PORTD,5);
			delay_us(200);
			PORT_OFF(PORTD,5);
			
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
		
		// Convert from revolutions per second to cm/sec
		v_l = M_2PIR*rps_lf;		
		v_r = M_2PIR*rps_rf;
		///////////////////////////////////////////////

		// Calculate total elapsed time ////////////////////////////
		elapsed_time = fmin(elapsed_time_r - elapsed_time_r_previous, elapsed_time_l - elapsed_time_l_previous);
		////////////////////////////////////////////////////////////

		/*
		// Calculate correction angle //////////////////////////////
		// The below code should not be in the final code and the commands
		// from the Upper Brain will be in the form and linear and angular 
		// velocity only

		cmd_lin_vel = 20;				

		elapsed_time = fmin(elapsed_time_r - elapsed_time_r_previous, elapsed_time_l - elapsed_time_l_previous);

		// Calculate robot angular velocity
		ang_vel_robot = (v_r - v_l) / ROBOT_DIAMETER;

		// Calculate robot heading
		heading_robot += ang_vel_robot * elapsed_time;

		// Limit the heading to 2pi
		heading_robot -= 2*M_PI*(1 + floor((heading_robot-M_PI)/(2*M_PI)));

		// Calculate angular velocity correction 
		cmd_ang_vel = -0.5*atan2(1*(heading_robot - cmd_heading) +0,1);
		////////////////////////////////////////////////////////////
		*/

		// Velocity Commands
		v_l_cmd = (cmd_lin_vel - cmd_ang_vel*ROBOT_RADIUS);
		v_r_cmd = (cmd_lin_vel + cmd_ang_vel*ROBOT_RADIUS);
		////////////////////////////////////////
		
		// PID calculation	/////////////////////////////////////////
		error_l = v_l_cmd - v_l;									// Current Left Wheel error
		error_r = v_r_cmd - v_r;									// Current Right Wheel error
	
		// Calculate the proportion
		up_l = Kp_l*error_l;										// Left Wheel Proportion
		up_r = Kp_r*error_r;										// Right Wheel Proportion
		
		// Calculate the integral
		ui_l = Ki_l*(error_l + error_l_n1) + ui_l_n1;				// Update Left Wheel integral
		ui_r = Ki_r*(error_r + error_r_n1) + ui_r_n1;				// Update Right Wheel integral

/*
		Td_l = 25*(elapsed_time);
		Td_r = 25*(elapsed_time);
		Kd_l = (2*K_l*Td_l)/T_l;									// Update Derivative coefficient
		Kd_r = (2*K_r*Td_r)/T_r;									// Update Derivative coefficient
*/
		// Calculate the derivative		
		ud_l = Kd_l*(error_l - error_l_n1) - ud_l_n1;				// Update Left Wheel derivative
		ud_r = Kd_r*(error_r - error_r_n1) - ud_r_n1;				// Update Right Wheel derivative
		
		// Filter the derivative output
		udf_l = lpf1_l*ud_l + lpf2_l*ud_l_n1 - lpf3_l*udf_l_n1;		// Update filtered Left Wheel derivative
		udf_r = lpf1_r*ud_r + lpf2_r*ud_r_n1 - lpf3_r*udf_r_n1;		// Update filtered Right Wheel derivative
		
		u_l += up_l;									// u_l(t) to be output to the Left Wheel 
		u_r += up_r;									// u_r(t) to be output to the Right Wheel 
		//u_l += up_l + ui_l + udf_l;									// u_l(t) to be output to the Left Wheel 
		//u_r += up_r + ui_r + udf_r;									// u_r(t) to be output to the Right Wheel 

		// Anti-windup
		if(u_l > 30){
			ui_l = ui_l_n1;
			u_l = 30;	
		}
		else if(u_l < -30){
			ui_l = ui_l_n1;
			u_l = -30;
		}

		if(u_r > 30){
			ui_r = ui_r_n1;
			u_r = 30;	
		}
		else if(u_r < -30){
			ui_r = ui_r_n1;
			u_r = -30;
		}

		error_l_n1 = error_l;						// Update previous Left Wheel proportional error
		ui_l_n1 = ui_l;								// Update previous Left Wheel integral
		ud_l_n1 = ud_l;								// Update previous Left Wheel derivative
		udf_l_n1 = udf_l;							// Update previous Left Wheel filtered derivative

		error_r_n1 = error_r;						// Update previous Right Wheel proportional error
		ui_r_n1 = ui_r;								// Update previous Right Wheel integral
		ud_r_n1 = ud_r;								// Update previous Right Wheel derivative
		udf_r_n1 = udf_r;							// Update previous Right Wheel filtered derivative

		/////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////
		
		
		wheel_l(u_l);
		wheel_r(u_r);
		

		// Update velocity commands /////////////////////////////////
		update_vw_cmd();
		/////////////////////////////////////////////////////////////

		//rprintfFloat(5,(elapsed_time_l - elapsed_time_l_previous));
		//rprintf("\t,");
		//rprintf("Left: ");
		//rprintfFloat(5,v_l);
		//rprintf("Right: ");
		//rprintf("\t,");
		//rprintfFloat(5,v_r);
		//rprintfFloat(5,cmd_ang_vel);
		//rprintf("\t,");
		//rprintfFloat(5,(elapsed_time_l - elapsed_time_l_previous));
		//rprintfCRLF();
	/**/	

		
	}
	while(1){
		rprintf("Dead.\n");
	}
	return 0;

}


