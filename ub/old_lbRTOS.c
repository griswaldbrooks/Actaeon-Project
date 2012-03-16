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
#define IR_TO_EDGE		10	//cm

#define NUM_IR_READS	10

float v_command = 0;	//wheel velocity command, used by PI controller, controlled by velocity filter
float inst_cmd_vel = 0;		//instantaneous velocity command manipulated by other functions, input to the vel filter
signed int v_offset = 0; //velocity offset for turning

int spray_time = 125;

//encoder measurements
s16 LEFTVel_current = 0;  //linear wheel velocity in cm/sec
s32 LEFTDis_current = 0;  //rolled distance in cm since power on
s32 LEFTDis_prev = 0;
s16 dLEFTDis = 0;		  //Dis_current - Dis_prev

s16 RIGHTVel_current = 0; //linear wheel velocity in cm/sec
s32 RIGHTDis_current = 0; //rolled distance in cm since power on
s32 RIGHTDis_prev = 0;
s16 dRIGHTDis = 0;		  //Dis_current - Dis_prev

s16 dRL;

float enc_heading = 0;
float dis_enc_heading;

s16 cmd_angle = 0; 	//angle in degrees
s32 cmd_dist = 0;	//commanded distance in cm

float correction_angle = 0;

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
			//	rprintf("RRR VVV: ");
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
	//posts commanded rotation and distance data to global variable
	static char lf_flag; //checks if line feed ("\n") character has been sent
	static char rot_iter; //count iterator for commanded rotation
	static char dis_iter; //count iterator for commanded distance
	static char rot_flag;
	static char dis_flag;
	static s16 rot_rough = 0;  //store ascii chars
	static s16 dis_rough = 0;  //store ascii chars
		c = c & 0b01111111;		//for some reason, every byte has its first bit set to 1
		uart1SendByte(c);
		if(c != 0xff){
		//if the data isn't whitespace (0xff), post it
		

			if(c == 0x0a){lf_flag = SET;} //line feed detected, the character will be a 'R' or a 'D'
	
			else if((lf_flag) && (c == 'R')){ //set rotation flag
				rot_flag = SET;
				rot_iter = 0;
				lf_flag = UNSET;
				//rprintf("R: char: %c ",c);
				return;
			} 
			else if((lf_flag) && (c == 'M')){ //set distance flag
				dis_flag = SET;
				dis_iter = 0;
				lf_flag = UNSET;
			//	rprintf("%c",c);
				return;
			}
			else if(rot_flag){
				rot_rough = (char2hex(c) | (rot_rough << 4));	//store then increment	
				rot_iter++;
				//rprintf(" #%c\t:%d\t",c,rot_iter);
				//rprintfu08(c); rprintf("\t");
				//rprintfu16(rot_rough);
				//rprintfCRLF();
			}	
			else if(dis_flag){
				dis_rough = (char2hex(c) | (dis_rough << 4));	//store then increment	
				dis_iter++;
			//	rprintf("%c",c);
			}

			if(rot_iter == 4){
				//cmd_angle = retConv_s16(&rot_rough);
				cmd_angle = rot_rough;
				rot_flag = UNSET;
				rot_iter = 0;
				//rprintfu16(rot_rough);
				//rprintf("\n\n");
				rot_rough = 0;
			}
			else if(dis_iter == 4){
				cmd_dist = dis_rough;
				dis_flag = UNSET;
				dis_iter = 0;
				dis_rough = 0;
			}
		
		}

		else{rprintf("WR\n");}
		
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
    uartSetBaudRate(0, 38400); // set UARTE speed, for Bluetooth
    uartSetBaudRate(1, 115200); // set UARTD speed, for USB connection, up to 500k, try 115200 if it doesn't work
    uartSetBaudRate(2, 38400); // set UARTH speed
    uartSetBaudRate(3, 38400); // set UARTJ speed, for Blackfin
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
	
	uartSetRxHandler(2, &fwdSer_L);
	uartSetRxHandler(0, &fwdSer_R);
	//uartSetRxHandler(3, &ubRcv);

	//UART ISR *** UART ISR ***

	LED_on();

	rprintf("\r\nSystem Warmed Up");

	// initialize the timer system
 	init_timer0(TIMER_CLK_1024);
// 	init_timer1(TIMER_CLK_64); // Timer 1 is initialized by FreeRTOS
 	init_timer2(TIMER2_CLK_64);
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


void wheel_L(signed int cmd_vel){
		if(cmd_vel > 36){cmd_vel = 36;}
		if(cmd_vel < -36){cmd_vel = -36;}
		
		float s_out = -0.4928 * (cmd_vel) + 185.27;
		
		taskENTER_CRITICAL();
		PWM_timer3_Set_E4((int)s_out);
		taskEXIT_CRITICAL();


}

void wheel_R(signed int cmd_vel){
		if(cmd_vel > 36){cmd_vel = 36;}
		if(cmd_vel < -36){cmd_vel = -36;}
				
		float s_out = -0.5421 * (cmd_vel) + 188.41;

		taskENTER_CRITICAL();
		PWM_timer3_Set_E3((int)s_out);
		taskEXIT_CRITICAL();


}

void vPID_L(void* pvParameters){
	
	portTickType xLastWakeTime;
	
	s16 error;
	s16 acc_error = 0;
	float KP = 0.25;
	float KI = 0.125;
	

	wheel_L_on();

	for(;;){
			signed int v_out = v_command + v_offset;
			error = v_out - LEFTVel_current;
			
			v_out += (signed int)((KP * error) + (KI * acc_error));
			acc_error += error;

			if(v_out > 36){v_out = 36;}
			if(v_out < -36){v_out = -36;}
			
			taskENTER_CRITICAL();
			wheel_L(v_out);
			taskEXIT_CRITICAL();
			
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));
	}

}

void vPID_R(void* pvParameters){
		
	portTickType xLastWakeTime;
	s16 error;
	s16 acc_error = 0;
	float KP = 0.25;
	float KI = 0.125;

	wheel_R_on();

	char adj;
	for(;;){
			
			signed int v_out = v_command - v_offset;
			error = v_command - (-RIGHTVel_current);
			v_out += (signed int)((KP * error) + (KI * acc_error));
			if(v_out > 36){v_out = 36;}
			if(v_out < -36){v_out = -36;}
			
			taskENTER_CRITICAL();
			wheel_R(-v_out);
			taskEXIT_CRITICAL();

			acc_error += error;
			
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));
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

void vRampVel(){
	const portTickType xTicksToWait = 1000 / portTICK_RATE_MS;
	portTickType xLastWakeTime;
	v_command = 0;
	inst_cmd_vel = 25;
	vTaskDelayUntil(&xLastWakeTime, 10000 / portTICK_RATE_MS);	//leave velocity at zero briefly

	for(;;){
		
		vTaskDelayUntil(&xLastWakeTime, xTicksToWait);
		//if(v_command != inst_cmd_vel){v_command++;}
		v_command += (1/(inst_cmd_vel - v_command))*(inst_cmd_vel);

		if(v_command > 25){v_command = 25;}
		if(v_command <  0){v_command =  0;}
		
	
	}
}

void vEnc_UpdatePose(){
	float enc_ang_vel = 0;
	float elapsed_time;

	for(;;){
		
		//The below formula yields the encoder calculated angular velocity of the robot
		//as it rotates about some ICC(Instantaneous Center of Curvature)
		reset_timer0();
		enc_ang_vel = ((-RIGHTVel_current) - LEFTVel_current) / ROBOT_DIAMETER;
		elapsed_time = ((get_timer0_overflow()*255 + TCNT0) * 0.256) / 1000;
		//dRL = dRIGHTDis - dLEFTDis;
		//dis_enc_heading += sin((dRL)/ ROBOT_DIAMETER);
		//dRIGHTDis = dLEFTDis = 0;
		enc_heading += enc_ang_vel * elapsed_time;


	}
}

void vUpdatePose(){
	//update the heading (orientation) of the robot
	//later this task will fuse encoder and acc/gyro data
	//and robot translation data
	s16 dDis;
	float CMD_K = 0.5;
	float COR_K = 1;

	for(;;){
		robot.heading = (180 * enc_heading) / M_PI; 
		v_offset = (0.5 * robot.heading) + CMD_K*cmd_angle + COR_K*correction_angle;
		robot.vel = (LEFTVel_current + (-RIGHTVel_current)) / 2;
		
		dRIGHTDis = (-RIGHTDis_current) - (-RIGHTDis_prev); 
		RIGHTDis_prev = RIGHTDis_current;
		dLEFTDis = LEFTDis_current - LEFTDis_prev;
		LEFTDis_prev = LEFTDis_current;
		
		dDis = (10*(dRIGHTDis + dLEFTDis))/2;	//Send shifted value to avoid decimals
		dRIGHTDis = dLEFTDis = 0;
		
		send_frame('H', robot.heading);
		send_frame('V', robot.vel);
		send_frame('S', dDis);

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
		rprintf("%d\n", cmd_angle);
		vTaskDelayUntil(&xLastWakeTime, xTicksToWait);
		cmd_angle = 90;
		rprintf("%d\n", cmd_angle);
		vTaskDelayUntil(&xLastWakeTime, xTicksToWait);
		cmd_angle = 0;
		rprintf("%d\n", cmd_angle);
		
	
	}
}



int main(void)
{
	prvSetupHardware();

	

	xTaskCreate(vLight0On, "Light0", 100, NULL, 1, NULL);
	xTaskCreate(vLight1On, "Light1", 100, NULL, 1, NULL);
	xTaskCreate(vRampVel, "RampVel", 100, NULL, 1, NULL);
	xTaskCreate(vUpdatePose, "UpdatePs", 500, NULL, 1, NULL);
	xTaskCreate(vEnc_UpdatePose, "enUpdtPs", 500, NULL, 1, NULL);
//	xTaskCreate(vServoOsc, "ServoGo", 200, NULL, 1, NULL);
//	xTaskCreate(vServoTm, "ServoTm", 200, NULL, 1, NULL);
	xTaskCreate(vPID_L, "vPID_L", 500, NULL, 2, NULL);
	xTaskCreate(vPID_R, "vPID_R", 500, NULL, 2, NULL);
	xTaskCreate(vScript, "vScript", 100, NULL, 2, NULL);


	vTaskStartScheduler();
	for(;;){rprintf("DEATH\n");}
	
	return 0;
}
