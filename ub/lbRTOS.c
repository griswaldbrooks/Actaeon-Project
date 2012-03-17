#include <SoR_Utils.h>

#define SET		0x01;
#define UNSET	0x00;
//#define TRUE	0x01;
//#define FALSE	0x00;


#define BUFFER_SIZE		256
uint8_t lds_buffer[BUFFER_SIZE];
uint8_t *lds_buffer_write_ndx = NULL;
uint8_t *lds_buffer_read_ndx = NULL;

typedef struct {
	uint8_t index;
	uint16_t motor_speed;
	uint8_t invalid[4];
	uint8_t strength_warning[4];
	uint16_t distance[4];
	uint16_t intensity[4];
	uint16_t checksum;

} LDS_FRAME;


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




/*
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
*/
void LDSRcv(unsigned char c){
	//if( (c != 0xff) && (lds_buffer_write_ndx != NULL) && (lds_buffer_write_ndx > lds_buffer_read_ndx)){
	if(c != 0xff){
		*lds_buffer_write_ndx = c;
		lds_buffer_write_ndx++;		
		if(lds_buffer_write_ndx >= (lds_buffer + BUFFER_SIZE)){ 
			lds_buffer_write_ndx = lds_buffer;
		}
	}

}

uint8_t read_LDS(){
	while(lds_buffer_read_ndx == lds_buffer_write_ndx){ delay_us(10); };
	uint8_t data = *lds_buffer_read_ndx;
	lds_buffer_read_ndx++;
	if(lds_buffer_read_ndx >= (lds_buffer + BUFFER_SIZE)){ 
		lds_buffer_read_ndx = lds_buffer;
	}
	return data;
}

uint8_t error_checker(const uint16_t chk_data[], uint16_t checksum){
    uint32_t chk32 = 0;
    for(uint8_t ndx = 0; ndx < 10; ndx++){
      chk32 = (chk32 << 1) + chk_data[ndx];
    }
    uint16_t chk16 = (chk32 & 0x7FFF) + (chk32 >> 15);
    chk16 = chk16 & 0x7FFF;
    return (chk16 == checksum);
  }

uint8_t parse_frame(LDS_FRAME *frame){
    // Array for use by the error checker
    uint16_t chk_data[10];

    // Read start byte
    uint8_t start_byte = read_LDS();
    // If the byte read in is not the start byte (0xFA), then this is not the beginning of a frame
    if(start_byte != 0xFA) return FALSE; 

    // Read index
	frame->index = read_LDS();
    // If the byte proceeding the start byte does not look like an index, then is is not the beginning of a frame
    if((frame->index < 0xA0)||(frame->index > 0xF9)) return FALSE;

    // Add these bytes for the error checker
    chk_data[0] = (frame->index << 8) + start_byte;

    // Read motor speed
    //   Motor speed is given as little endian from the LDS, but by reading the motor speed into a uint16_t rather
    //   than a uint8_t array, the bits are ordered correctly. The resultant uint16_t (motor_speed) is a fixed point number,
    //   structured as follows:
    //   MSB                                                               LSB
    //   15  14  13  12  11  10  9   8   7   6    5    4    3    2    1    0
    //   512 256 128 64  32  16  8   4   2   1 .  1/2  1/4  1/8  1/16 1/32 1/64
    //
	
	frame->motor_speed = ((uint16_t)read_LDS()) + ((uint16_t)read_LDS()<<8);
    // Add these bytes for the error checker
    chk_data[1] = frame->motor_speed;
	
				
    // Read distances, intensities, and flags
    for(uint8_t itr = 0; itr < 4; itr++) {
      // Read in the four distance and intensity bytes
      uint8_t di_bytes[4];
	  for(uint8_t d_ndx = 0; d_ndx < 4; d_ndx++){
	  	di_bytes[d_ndx] = read_LDS();
      }
      // The invalid and strength flags are the MSB and adjacent bit of the second (indexwise) distance byte
      frame->invalid[itr] = (uint8_t)(di_bytes[1] & 0x80);
      frame->strength_warning[itr] = (uint8_t)(di_bytes[1] & 0x40);

      // Distance and intensity bytes are read in little endian, and therefore need to be switched and concatenated
      // The second distance byte (indexwise) is masked to eliminate the invalid and strength flags from the measurement
      if(frame->invalid[itr]){ // If this measurement is invalid, set it to zero
		frame->distance[itr]  = 0;
      }
      else{
		frame->distance[itr]  = ((di_bytes[1] & 0x3F) << 8) + di_bytes[0];
      }

      frame->intensity[itr] = (di_bytes[3] << 8) + di_bytes[2];
      
      // Add these bytes for the error checker
      chk_data[2*itr + 2] = (di_bytes[1] << 8) + di_bytes[0];
      chk_data[2*itr + 3] = (di_bytes[3] << 8) + di_bytes[2];
    }

    // Read checksum
    // Checksum is given in little endian, but because it is read into a uint16_t, it is ordered correctly
	frame->checksum = (((uint16_t)read_LDS())) + (((uint16_t)read_LDS())<<8);
    
    return error_checker(chk_data, frame->checksum);
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
    uartSetBaudRate(3, 115200); // set UARTJ speed, for Blackfin
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
	
//	uartSetRxHandler(2, &fwdSer_L);
//	uartSetRxHandler(0, &fwdSer_R);
	uartSetRxHandler(3, &LDSRcv);

	//UART ISR *** UART ISR ***

	LED_on();

	rprintf("\r\nSystem Warmed Up");

	// initialize the timer system
 	init_timer0(TIMER_CLK_1024);
 	init_timer1(TIMER_CLK_64); // Timer 1 is initialized by FreeRTOS
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


void print_frame(LDS_FRAME frame){
	rprintf("Index: ");
	rprintfu08(frame.index);
	rprintf(" Motor Speed: ");
	rprintfu16(frame.motor_speed);
	rprintf(" Distances: ");
	for(uint8_t ndx = 0; ndx < 4; ndx++){
		rprintf("%d ",frame.distance[ndx]);
	}
	rprintfCRLF();
}

void grab_frames(const LDS_FRAME *frame, LDS_FRAME ldsf[]){
	LDS_FRAME t_frame;
	uint8_t f_ndx = 1;
	ldsf[0] = *frame;

	while(f_ndx < 90){
		if(parse_frame(&t_frame)){
			ldsf[f_ndx] = t_frame;
			f_ndx++;
		}
	}
}

void conv_FrametoDist(const LDS_FRAME ldsf[], uint16_t ranges[]){
	// ldsf must be of length 90
	// ranges must be of length 360
	for(uint16_t f_ndx = 0; f_ndx < 90; f_ndx++){
		ranges[4*f_ndx]     = (ldsf[f_ndx].distance[0]);
		ranges[4*f_ndx + 1] = (ldsf[f_ndx].distance[1]);
		ranges[4*f_ndx + 2] = (ldsf[f_ndx].distance[2]);
		ranges[4*f_ndx + 3] = (ldsf[f_ndx].distance[3]);
	}
}



int main(void)
{
	LDS_FRAME frame;
	LDS_FRAME ldsf[90];
	uint16_t ranges[360];

	lds_buffer_write_ndx = lds_buffer_read_ndx = lds_buffer;
	prvSetupHardware();
	rprintf("Starting program.\n");

	while(1){
	// Grab frames
		if(parse_frame(&frame)){
			//rprintf("Got frame.\n");
			// If you get the first frame, start reading into the frame buffer
			if(frame.index == 0xA0){
				grab_frames(&frame,ldsf);
				rprintf("Frames grabbed.\n");
				// Convert frames into distances
				conv_FrametoDist(ldsf,ranges);
				rprintf("Frames Converted.\n");
				// Print ranges
				rprintf("Scan start\n");
				for(uint16_t r_ndx = 0; r_ndx < 360; r_ndx++){
					rprintf("%d",ranges[r_ndx]);
					rprintfCRLF();
				}
				rprintf("\nScan end\n\n");
			}
		}


	}

	rprintf("Program terminated.");
	
	return 0;
}
