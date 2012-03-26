#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <SoR_Utils.h>

#define SET		0x01;
#define UNSET	0x00;

#define wheel_L_on()	PWM_timer3_On_E4()
#define wheel_R_on()	PWM_timer3_On_E3()
#define spray_L_on()	PWM_timer4_On_H3()
#define spray_R_on()	PWM_timer4_On_H4()

#define wheel_L_off()	PWM_timer3_Off_E4()
#define wheel_R_off()	PWM_timer3_Off_E3()
#define spray_L_off()	PWM_timer4_Off_H3()
#define spray_R_off()	PWM_timer4_Off_H4()


//there are 128 ticks per rotation
#define CM_TICK		0.25 // cm/tick
//wheel radius
#define WH_RADIUS	5.08 //cm
//robot radius
#define ROBOT_RADIUS	8.2 //cm
#define ROBOT_DIAMETER	16.8 //cm

int spray_time = 125;

//encoder measurements
s16 LEFTVel_current = 0;  	//linear wheel velocity in cm/sec
u08 LEFTVel_ready = UNSET;	//left velocity ready flag
s32 LEFTDis_current = 0;  	//rolled distance in cm since power on
s32 LEFTDis_prev = 0;
s16 dLEFTDis = 0;		  	//Dis_current - Dis_prev

s16 RIGHTVel_current = 0; 	//linear wheel velocity in cm/sec
u08 RIGHTVel_ready = UNSET; //right velocity ready flag
s32 RIGHTDis_current = 0; 	//rolled distance in cm since power on
s32 RIGHTDis_prev = 0;
s16 dRIGHTDis = 0;		  	//Dis_current - Dis_prev

s16 dRL;

float enc_heading = 0;
float enc_ang_vel = 0;	//Angular velocity of robot
float dis_enc_heading;

float v_command = 0;	//wheel velocity command, used by PI controller, controlled by velocity filter

float cmd_angle = 0; 	//angle in radians
float cmd_ang_vel = 0; //commanded angular velocity in radians/sec
s32 cmd_dist = 0;	//commanded distance in cm

union u_vel{
	float f_vel;
	uint8_t arr_vel[4];
};

union u_ome{
	float f_ome;
	uint8_t arr_ome[4];
};

#define UB_BUFFER_SIZE	512
uint8_t ub_buffer[UB_BUFFER_SIZE];
uint8_t *ub_buffer_write_ndx = NULL;
uint8_t *ub_buffer_read_ndx = NULL;

typedef struct {
	float heading;
	float vel;
	u16 x;
	u16 y;
} R_POSE;

R_POSE robot;

//-----------------------------**&&**


void prvPWMSetup(){

	PWM_Init_timer3_E4(10); // Left wheel
	PWM_Init_timer3_E3(10); // Right wheel
	PWM_Init_timer4_H3(10);	// Left sprayer
	PWM_Init_timer4_H4(10);	// Right sprayer

}

char char2hex(char c){
	
	if((c > 47) && (c <58)){
		c -= 48;
	}
	else if((c > 64) && (c <71)){
		c -= 55;
	}
	return c;
}

s16 retConv_s16(char* ch_head){

	s16 int_val = 0x0000;
	char iv1 = *(ch_head);
	char iv2 = *(ch_head + 1);
	char iv3 = *(ch_head + 2);
	char iv4 = *(ch_head + 3);
	
	iv1 = char2hex(iv1);
	iv2 = char2hex(iv2);
	iv3 = char2hex(iv3);
	iv4 = char2hex(iv4);

	int_val = (int_val | iv1);
	int_val = ((int_val<<4) | iv2);
	int_val = ((int_val<<4) | iv3);
	int_val = ((int_val<<4) | iv4);
	return int_val;
}

s32 retConv_s32(char* ch_head){

	s32 int_val = 0x00000000;
	char iv1 = *(ch_head);
	char iv2 = *(ch_head + 1);
	char iv3 = *(ch_head + 2);
	char iv4 = *(ch_head + 3);
	char iv5 = *(ch_head + 4);
	char iv6 = *(ch_head + 5);
	char iv7 = *(ch_head + 6);
	char iv8 = *(ch_head + 7);
	
	iv1 = char2hex(iv1);
	iv2 = char2hex(iv2);
	iv3 = char2hex(iv3);
	iv4 = char2hex(iv4);
	iv5 = char2hex(iv5);
	iv6 = char2hex(iv6);
	iv7 = char2hex(iv7);
	iv8 = char2hex(iv8);

	int_val = (int_val | iv1);
	int_val = ((int_val<<4) | iv2);
	int_val = ((int_val<<4) | iv3);
	int_val = ((int_val<<4) | iv4);
	int_val = ((int_val<<4) | iv5);
	int_val = ((int_val<<4) | iv6);
	int_val = ((int_val<<4) | iv7);
	int_val = ((int_val<<4) | iv8);
	return int_val;
}


void fwdSer_R(unsigned char c){
//posts velocity and distance data to global variable
	static char lf_flag; //checks if line feed ("\n") character has been sent
	static char v_iter; //count iterator for vel_rough
	static char d_iter; //count iterator for dis_rough
	static char v_flag;
	static char d_flag;
	static char vel_rough[4];  //store ascii chars
	static char dis_rough[8];  //store ascii chars

		//rprintf("%c",c);
		if(c != 0xff){
		//if the data isn't whitespace (0xff), post it
		//	rprintf("%c",c);

			if(c == 0x0A){lf_flag = SET;} //line feed detected, the character will be a 'D' or a 'V'
	
			else if((lf_flag) && (c == 'V')){ //set velocity flag
				v_flag = SET;
				v_iter = 0;
				lf_flag = UNSET;
				return;
			} 
			else if((lf_flag) && (c == 'D')){ //set distance flag
				d_flag = SET;
				d_iter = 0;
				lf_flag = UNSET;
			//	rprintf("%c",c);
				return;
			}
			else if(v_flag){
				vel_rough[v_iter++] = c;	//store then increment	
				//rprintf("%c",c);
			}	
			else if(d_flag){
				dis_rough[d_iter++] = c;	//store then increment	
			//	rprintf("%c",c);
			}

			if(v_iter == 4){
				RIGHTVel_current = CM_TICK * retConv_s16(&vel_rough);
				v_flag = UNSET;
				v_iter = 0;
				RIGHTVel_ready = SET;

			//	rprintf("RRR VVV: ");
			//	rprintfu16(RIGHTVel_current);
			//	rprintf("\n");
			}
			else if(d_iter == 8){
				RIGHTDis_current = CM_TICK * retConv_s32(&dis_rough);
				//rprintf("dR: %d\n",dRIGHTDis);
				d_flag = UNSET;
				d_iter = 0;
				//rprintf("RRR DDD: ");
				//rprintfFloat(5,RIGHTDis_current);
				//rprintf("\n");
			}
		
		}

		else{rprintf("WR\n");}

	//	if(xHigherPriorityTaskWoken == pdTRUE){
		//if data couldn't be posted, rprintf a message
		
			//taskYIELD();
	//	}
		
}
void fwdSer_L(unsigned char c){
//posts velocity and distance data to global variable
	static char lf_flag; //checks if line feed ("\n") character has been sent
	static char v_iter; //count iterator for vel_rough
	static char d_iter; //count iterator for dis_rough
	static char v_flag;
	static char d_flag;
	static char vel_rough[4];  //store ascii chars
	static char dis_rough[8];  //store ascii chars

		//rprintf("%c",c);
		if(c != 0xff){
		//if the data isn't whitespace (0xff), post it
		//	rprintf("%c",c);

			if(c == 0x0A){lf_flag = SET;} //line feed detected, the character will be a 'D' or a 'V'
	
			else if((lf_flag) && (c == 'V')){ //set velocity flag
				v_flag = SET;
				v_iter = 0;
				lf_flag = UNSET;
				return;
			} 
			else if((lf_flag) && (c == 'D')){ //set distance flag
				d_flag = SET;
				d_iter = 0;
				lf_flag = UNSET;
			//	rprintf("%c",c);
				return;
			}
			else if(v_flag){
				vel_rough[v_iter++] = c;	//store then increment	
				//rprintf("%c",c);
			}	
			else if(d_flag){
				dis_rough[d_iter++] = c;	//store then increment	
			//	rprintf("%c",c);
			}

			if(v_iter == 4){
				LEFTVel_current = CM_TICK * retConv_s16(&vel_rough);
				v_flag = UNSET;
				v_iter = 0;
				LEFTVel_ready = SET;

			//	rprintf("LEFTVel_ready: %d\n", LEFTVel_ready);
			//	rprintf("LLL VVV: ");
			//	rprintfu16(RIGHTVel_current);
			//	rprintf("\n");
			}
			else if(d_iter == 8){
				LEFTDis_current = CM_TICK * retConv_s32(&dis_rough);
			//	rprintf("dL: %d\n",dLEFTDis);
				d_flag = UNSET;
				d_iter = 0;
			//	rprintf("LLL DDD: ");
			//	rprintfu32(LEFTDis_current);
			//	rprintf("\n");
			}
		
		}

		else{rprintf("WR\n");}

	//	if(xHigherPriorityTaskWoken == pdTRUE){
		//if data couldn't be posted, rprintf a message
		
			//taskYIELD();
	//	}
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

void prvSetupHardware(){

	int i, j;

	//add 1.7s delay for potential power issues
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	delay_cycles(65535);
	
	

	uartInit();  // initialize the UART (serial port)
    uartSetBaudRate(0, 38400); // Right Encoder
    uartSetBaudRate(1, 115200); // USB connection
    uartSetBaudRate(2, 38400); // Left Encoder
    uartSetBaudRate(3, 38400); // UB connection
	//G=Ground, T=Tx (connect to external Rx), R=Rx (connect to external Tx)

	rprintfInit(uart1SendByte);// initialize rprintf system and configure uart1 (USB) for rprintf

	configure_ports(); // configure which ports are analog, digital, etc.
	
	//I2C init
	/*
	i2cInit();  //initialize I2C interface
	i2cSetBitrate(100);  //set I2C transaction bit rate in kHz
	//set local device address and allow
	// response to general call
	i2cSetLocalDeviceAddr(0x02, TRUE); //i2cSetLocalDeviceAddr(LOCAL_ADDR, TRUE);
	i2cSetSlaveReceiveHandler(i2cSlaveReceiveService);
	i2cSetSlaveTransmitHandler(i2cSlaveTransmitService);
	*/

	
	//UART ISR *** UART ISR ***

	init_ub_buffer();
	
	uartSetRxHandler(2, &fwdSer_L);
	uartSetRxHandler(0, &fwdSer_R);
	uartSetRxHandler(3, &ubRcv);

	//UART ISR *** UART ISR ***

	LED_on();

	rprintf("\r\nSystem Warmed Up");

	// initialize the timer system
 	init_timer0(TIMER_CLK_1024);
// 	init_timer1(TIMER_CLK_64); // Timer 1 is initialized by FreeRTOS
 	//init_timer2(TIMER2_CLK_64);
	init_timer2(TIMER2_CLK_1024);
 	init_timer3(TIMER_CLK_64);
 	init_timer4(TIMER_CLK_64);
 	init_timer5(TIMER_CLK_64);

	a2dInit(); // initialize analog to digital converter (ADC)
	a2dSetPrescaler(ADC_PRESCALE_DIV32); // configure ADC scaling
	a2dSetReference(ADC_REFERENCE_AVCC); // configure ADC reference voltage

	//let system stabelize for X time
	for(i=0;i<16;i++)
		{
		j=a2dConvert8bit(i);//read each ADC once to get it working accurately
		delay_cycles(5000); //keep LED on long enough to see Axon reseting
		rprintf(".");
		}

	LED_off();

	rprintf("Initialization Complete \r\n");

	//reset all timers to zero
	reset_timer0();
	reset_timer1();
	reset_timer2();
	reset_timer3();
	reset_timer4();
	reset_timer5();



	/********PWM Setup***********/
	prvPWMSetup();

}

/*************************************************/


void wheel_L(float cmd_vel){
		if(cmd_vel > 36){cmd_vel = 36;}
		if(cmd_vel < -36){cmd_vel = -36;}
		
		float s_out = -0.4928 * (cmd_vel) + 185.27;
		
		taskENTER_CRITICAL();
		PWM_timer3_Set_E4((int)s_out);
		taskEXIT_CRITICAL();


}

void wheel_R(float cmd_vel){
		if(cmd_vel > 36){cmd_vel = 36;}
		if(cmd_vel < -36){cmd_vel = -36;}
				
		float s_out = -0.5421 * (cmd_vel) + 188.41;

		taskENTER_CRITICAL();
		PWM_timer3_Set_E3((int)s_out);
		taskEXIT_CRITICAL();


}


void vPID(void* pvParameters){
	
	portTickType xLastWakeTime;
	
	float error_L;
	float acc_error_L = 0;
	float error_R;
	float acc_error_R = 0;

	const float KP_L = 2*3*(0.9/(796.1475*0.1206));
	const float TI_L = (0.1206/0.3);
	const float KI_L = KP_L/TI_L;

	const float KP_R = 2*2.5*(0.9/(676.1129*0.1597));
	const float TI_R = (0.1597/0.3);
	const float KI_R = KP_R/TI_R;
	
	float v_out_L = 0;		//velocity output to left wheel
	float v_out_R = 0;		//velocity output to right wheel

	float v_left = 0;	//velocity command to left wheel
	float v_right = 0;	//velocity command to right wheel

	float dt = 0;
	
	wheel_L_on();
	wheel_R_on();

	reset_timer2();

	for(;;){
	
	// Forward Kinematics
	// Velocity Adjustment
	/*
		if((1 - (pow(cmd_ang_vel,2))/(pow(cmd_ang_vel,2) + 0.0625)) < 1){
			v_left = (signed int)(v_command*(1 - (pow(cmd_ang_vel,2))/(pow(cmd_ang_vel,2) + 0.0625)) - cmd_ang_vel*ROBOT_RADIUS);
			v_right = (signed int)(v_command*(1 - (pow(cmd_ang_vel,2))/(pow(cmd_ang_vel,2) + 0.0625)) + cmd_ang_vel*ROBOT_RADIUS);
		}
		else{
			v_left = (signed int)(v_command - cmd_ang_vel*ROBOT_RADIUS);
			v_right = (signed int)(v_command + cmd_ang_vel*ROBOT_RADIUS);
		}
	*/
	
		v_left = (v_command - cmd_ang_vel*ROBOT_RADIUS);
		v_right = (v_command + cmd_ang_vel*ROBOT_RADIUS);

		dt = ((get_timer2_overflow()*255 + TCNT2) * 0.0633) / 1000;
		reset_timer2();
	
	// PID

		LEFTVel_ready = RIGHTVel_ready = UNSET;

		error_L = v_left - LEFTVel_current;
		error_R = v_right - (-RIGHTVel_current);
			
		v_out_L = KP_L*error_L + KI_L*acc_error_L + (2/25)*v_left;
		v_out_R = KP_R*error_R + KI_R*acc_error_R + (2/25)*v_right;
		
		acc_error_L += error_L*dt;
		acc_error_R += error_R*dt;
/*
			taskENTER_CRITICAL();
//			rprintf("%d,\t", LEFTVel_current);
//			rprintfFloat(5,LEFT_Vf);
			rprintf("LEFTVel: %d\tv_out_L: %d\t", LEFTVel_current, (int)v_out_L);
			rprintf("RIGHTVel: %d\tv_out_R: %d\t",-RIGHTVel_current,(int)v_out_R);
			rprintf("cmd_ang_vel: ");
			rprintfFloat(5,cmd_ang_vel);
			rprintf("\tenc_heading: ");
			rprintfFloat(5,enc_heading);
			rprintf("\tdt: ");
			rprintfFloat(8,dt);
			rprintfCRLF();
			taskEXIT_CRITICAL();
*/
			
		taskENTER_CRITICAL();
		wheel_L(v_out_L);
		wheel_R(-v_out_R);
		PORT_ON(PORTL,1);
		delay_us(50);
		PORT_OFF(PORTL,1);
		taskEXIT_CRITICAL();

		vTaskDelayUntil(&xLastWakeTime, (1 / portTICK_RATE_MS));
	}

}

void vLight0On(void *pvParameters){
	portTickType xLastWakeTime;
	for(;;){
		PORT_ON(PORTA, 0);
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));
		//taskYIELD();	
		PORT_OFF(PORTA, 0);
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));
		
	}
}
void vLight1On(void *pvParameters){
	portTickType xLastWakeTime;
	for(;;){
		LED_on();
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));
		LED_off();
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));	}
}

void vServoTm(){
	char adj;
	for(;;){
		adj = uart1GetByte();
		if(adj == 'p'){ spray_time++;}
		else if(adj == 'l'){ spray_time--;}
		rprintf("%d\n", spray_time);
	}

}


void vServoOsc(){
	portTickType xLastWakeTime;
	
	uint8_t adj;
	spray_R_on();
	spray_L_on();
	uint16_t h3duty = 122;

	for(;;){
		if(h3duty > 1023){ h3duty = 0;}
		taskENTER_CRITICAL();
	//	PWM_timer3_Set_E3(i);	//right
		PWM_timer4_Set_H3(h3duty);	//left sprayer //originally 204
		PWM_timer4_Set_H4(204);	//right sprayer
		taskEXIT_CRITICAL();
		vTaskDelayUntil(&xLastWakeTime, (spray_time / portTICK_RATE_MS));
		adj = uart1GetByte();
		if(adj == 'p'){ h3duty++;;}
		else if(adj == 'l'){ h3duty--;}
		/*taskENTER_CRITICAL();
		PWM_timer4_Set_H3(135);	
		PWM_timer4_Set_H4(135);	
		taskEXIT_CRITICAL();
		vTaskDelayUntil(&xLastWakeTime, (spray_time / portTICK_RATE_MS));
		*/

		rprintf("%d", h3duty);
		//rprintf(",");
		//rprintfNum(10, 5, 1, ' ', LEFTVel_current);
		rprintf("\n");
	
		h3duty++;

			
		

		
	}
}



char num2char(char c){
	
	if(c <10){
		c += 48;
	}
	else if((c >= 10) && (c <= 16)){
		c += 55;
	}
	return c;
}


void send_frame(char flag, int16_t data){
//send data frame
	uint8_t r1 = 0;
	uint8_t r2 = 0;
	uint8_t r3 = 0;
	uint8_t r4 = 0;

	uart3SendByte(flag);

	r1 = num2char(0x0F & (uint8_t)data);
	r2 = num2char(0x0F & ((uint8_t)(data >> 4)) );
	r3 = num2char(0x0F & ((uint8_t)(data >> 8)) );
	r4 = num2char(0x0F & ((uint8_t)(data >> 12)) );

	uart3SendByte(r4);
	uart3SendByte(r3);
	uart3SendByte(r2);
	uart3SendByte(r1);
	uart3SendByte('\n'); //line feed
}



void vEnc_UpdatePose(){
	portTickType xLastWakeTime;
	float elapsed_time;
	reset_timer0();

	for(;;){
		if(LEFTVel_ready || RIGHTVel_ready){
		//The below formula yields the encoder calculated angular velocity of the robot
		//as it rotates about some ICC(Instantaneous Center of Curvature)
		
		enc_ang_vel = ((-RIGHTVel_current) - LEFTVel_current) / ROBOT_DIAMETER;
		
		//elapsed_time = ((get_timer0_overflow()*255 + TCNT0) * 0.256) / 1000;
		elapsed_time = ((get_timer0_overflow()*255 + TCNT0) * 0.0633) / 1000;
		reset_timer0();
		//dRL = dRIGHTDis - dLEFTDis;
		//dis_enc_heading += sin((dRL)/ ROBOT_DIAMETER);
		//dRIGHTDis = dLEFTDis = 0;
		enc_heading += enc_ang_vel * elapsed_time;
		// Limit the heading to 2pi
		enc_heading -= 2*M_PI*(1 + floor((enc_heading-M_PI)/(2*M_PI)));
		//cmd_ang_vel = 3*pow(cmd_angle - enc_heading,2)/(pow(cmd_angle - enc_heading,2) + pow(3,2));
		cmd_ang_vel = -0.5*atan2(1*(enc_heading - cmd_angle) +0,1);
		/*
		rprintf("cmd_ang_vel: ");
		rprintfFloat(5,cmd_ang_vel);
		rprintfCRLF();
		rprintf("enc_heading: ");
		rprintfFloat(5,enc_heading);
		rprintfCRLF();
		
		taskENTER_CRITICAL();
		rprintf("elapsed_time: ");
		rprintfFloat(10,elapsed_time);
		rprintfCRLF();
		taskEXIT_CRITICAL();
		*/

		}
		
	vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));
	}
}

void vUpdatePose(){
	//update the heading (orientation) of the robot
	//later this task will fuse encoder and acc/gyro data
	//and robot translation data
	s16 dDis;
	float CMD_K = 0.5;
	float COR_K = 1;
	uint8_t lds_char;

	for(;;){
		robot.heading = (180 * enc_heading) / M_PI; 
		robot.vel = (LEFTVel_current + (-RIGHTVel_current)) / 2;
		
		dRIGHTDis = (-RIGHTDis_current) - (-RIGHTDis_prev); 
		RIGHTDis_prev = RIGHTDis_current;
		dLEFTDis = LEFTDis_current - LEFTDis_prev;
		LEFTDis_prev = LEFTDis_current;
		
		dDis = (10*(dRIGHTDis + dLEFTDis))/2;	//Send shifted value to avoid decimals
		dRIGHTDis = dLEFTDis = 0;
		
		//send_frame('H', robot.heading);
		//send_frame('V', robot.vel);
		//send_frame('S', dDis);

//		lds_char = read_LDS();

//		if(lds_char == 0xFA) rprintfCRLF();
		
//		uart1SendByte(num2char(0x0F & (lds_char >> 4)));
//		uart1SendByte(num2char(0x0F & lds_char));


		//rprintf("r:%d,w:%d\n",lds_buffer_read_ndx,lds_buffer_write_ndx);

/*
		rprintf("H: ");
		rprintfFloat(5, robot.heading);
		rprintf("\tV: ");
		rprintfFloat(5, robot.vel);

		
		rprintf("\t cmd_angle: %d\t", cmd_angle);
		rprintf("dDis: %d  ",dDis);
		//rprintfFloat(5, (180 * dis_enc_heading) / M_PI);
		//rprintfu16(cmd_angle);
		rprintfCRLF();
*/
		//rprintfu16((u16)robot.heading);
		/*
		rprintfFloat(10, robot.heading);
		rprintf("\t");
		rprintfFloat(10, cmd_angle);
		rprintf("\t%d", v_offset);
		rprintfCRLF();
		*/
		
	}

}

void vScript(){
	const portTickType xTicksToWait = 10000 / portTICK_RATE_MS;
	portTickType xLastWakeTime;

	for(;;){
		cmd_angle = 0;
		rprintf("cmd_angle: ");
		rprintfFloat(5, cmd_angle);
		rprintfCRLF();
		vTaskDelayUntil(&xLastWakeTime, xTicksToWait);
		cmd_angle = 90*(M_PI/180);
		rprintf("cmd_angle: ");
		rprintfFloat(5, cmd_angle);
		rprintfCRLF();
		vTaskDelayUntil(&xLastWakeTime, xTicksToWait);
		cmd_angle = 0;
		//rprintf("%d\n", cmd_angle)
		
	
	}
}

void vUbRcv(){
	portTickType xLastWakeTime;
	union u_vel fltuint8_velocity;
	union u_ome fltuint8_omega;
	uint8_t rcv_byte;
	uint8_t rcv_chk;
	uint8_t calc_chk;
	for(;;){
		rcv_byte = read_ub();
		//rprintfu08(rcv_byte);
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
/**/
			rprintfu08(fltuint8_velocity.arr_vel[0]);
			rprintfu08(fltuint8_velocity.arr_vel[1]);
			rprintfu08(fltuint8_velocity.arr_vel[2]);
			rprintfu08(fltuint8_velocity.arr_vel[3]);

			rprintfu08(fltuint8_omega.arr_ome[0]);
			rprintfu08(fltuint8_omega.arr_ome[1]);
			rprintfu08(fltuint8_omega.arr_ome[2]);
			rprintfu08(fltuint8_omega.arr_ome[3]);

			if(calc_chk == rcv_chk){
			}
				v_command = fltuint8_velocity.f_vel;
				cmd_ang_vel = fltuint8_omega.f_ome;
				
				
				rprintfCRLF();
				rprintf("Received Velocity: ");
				rprintfFloat(5,fltuint8_velocity.f_vel);
				rprintf(" Omega: ");
				rprintfFloat(5,fltuint8_omega.f_ome);
				rprintfCRLF();
			
			
		}
		vTaskDelayUntil(&xLastWakeTime, (1 / portTICK_RATE_MS));
	}
}

char key;
float elapsed_time;
float current_time = 0;
float previous_time = 0;

int main(void)
{
	prvSetupHardware();

	
	//delay_ms(500);


	v_command = 25;
//	v_command = 0;
	cmd_ang_vel = 0;


	xTaskCreate(vLight0On, "Light0", 100, NULL, 1, NULL);
	xTaskCreate(vLight1On, "Light1", 100, NULL, 1, NULL);
	xTaskCreate(vUpdatePose, "UpdatePs", 500, NULL, 1, NULL);
	xTaskCreate(vEnc_UpdatePose, "enUpdtPs", 500, NULL, 1, NULL);
//	xTaskCreate(vServoOsc, "ServoGo", 200, NULL, 1, NULL);
//	xTaskCreate(vServoTm, "ServoTm", 200, NULL, 1, NULL);
//	xTaskCreate(vUbRcv, "vUbRcv", 500, NULL, 1, NULL);
	xTaskCreate(vPID, "vPID", 500, NULL, 1, NULL);
//	xTaskCreate(vScript, "vScript", 100, NULL, 2, NULL);


	vTaskStartScheduler();
	for(;;){rprintf("DEATH\n");}
	
	return 0;
}
