
#include <SoR_Utils.h>
#include <lds.h>
#include <wall_follow.h>


#define SET				0x01
#define UNSET			0x00
#define AUDIO_PIN		10
#define AUDIO_THRESH	613
#define PEAKS_IN_10ms	30
// There should be 38 peaks in 10 ms

union u_vel{
	float f_vel;
	uint8_t arr_vel[4];
};

union u_ome{
	float f_ome;
	uint8_t arr_ome[4];
};

typedef struct {
	float heading;
	float vel;
	u16 x;
	u16 y;
} R_POSE;

R_POSE robot;

//-----------------------------**&&**


void lbRcv(unsigned char c){
		
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
    uartSetBaudRate(0, 38400); // LB
    uartSetBaudRate(1, 115200); // USB
    uartSetBaudRate(2, 115200); // XBEE
    uartSetBaudRate(3, 115200); // LDS
	//G=Ground, T=Tx (connect to external Rx), R=Rx (connect to external Tx)

	rprintfInit(uart1SendByte);// initialize rprintf system and configure uart1 (USB) for rprintf
	//rprintfInit(uart1SendByte);// initialize rprintf system and configure uart1 (USB) for rprintf

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
	
	uartSetRxHandler(0, &lbRcv);
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


}

/*************************************************/



void send_frame(float velocity, float omega){
//send data frame
	
	// Break the floats into 4 bytes
	union u_vel fltuint8_velocity;
	union u_ome fltuint8_omega;
	fltuint8_velocity.f_vel = velocity;
	fltuint8_omega.f_ome = omega;
		
	// Send header
	uart0SendByte(0xFA);
	//rprintfu08(0xFA);

	// Send linear velocity
	uart0SendByte(fltuint8_velocity.arr_vel[0]);
	uart0SendByte(fltuint8_velocity.arr_vel[1]);
	uart0SendByte(fltuint8_velocity.arr_vel[2]);
	uart0SendByte(fltuint8_velocity.arr_vel[3]);

	//rprintfu08(fltuint8_velocity.arr_vel[0]);
	//rprintfu08(fltuint8_velocity.arr_vel[1]);
	//rprintfu08(fltuint8_velocity.arr_vel[2]);
	//rprintfu08(fltuint8_velocity.arr_vel[3]);

	// Send angular velocity
	uart0SendByte(fltuint8_omega.arr_ome[0]);
	uart0SendByte(fltuint8_omega.arr_ome[1]);
	uart0SendByte(fltuint8_omega.arr_ome[2]);
	uart0SendByte(fltuint8_omega.arr_ome[3]);
/*
	rprintfu08(fltuint8_omega.arr_ome[0]);
	rprintfu08(fltuint8_omega.arr_ome[1]);
	rprintfu08(fltuint8_omega.arr_ome[2]);
	rprintfu08(fltuint8_omega.arr_ome[3]);
*/
	// Send checksum
	uint8_t chk = fltuint8_velocity.arr_vel[0] + fltuint8_velocity.arr_vel[1] + fltuint8_velocity.arr_vel[2]
	+ fltuint8_velocity.arr_vel[3] + fltuint8_omega.arr_ome[0] + fltuint8_omega.arr_ome[1] 
	+ fltuint8_omega.arr_ome[2] + fltuint8_omega.arr_ome[3];

	uart0SendByte(chk);
	//rprintfu08(chk);
}

void right_wall(uint16_t range[]){
	float lin_v = 0;
	float ang_v = 0;
	uint8_t f_cntr = 0;
	uint8_t r_cntr = 0;
	float f_rng_avg = 0;
	float r_rng_avg = 0;
	const uint8_t MIN_RANGE = 150; // mm
	// Calculate the average of the front 5 valid beams
	for(uint8_t r_ndx = 0; r_ndx < 5; r_ndx++){
		uint8_t f_ndx = 358 + r_ndx;
		// Accomodate wrap around
		if(f_ndx > 359){
			f_ndx -= 360;
		}
		if(range[f_ndx] > MIN_RANGE){
			f_rng_avg += (float)range[f_ndx];
			f_cntr++;
		}
	}
	if(f_cntr != 0){
		f_rng_avg = f_rng_avg/(float)f_cntr;
	}
//	rprintf("Front Range: ");
//	rprintfFloat(5, f_rng_avg);
//	rprintfCRLF();
	// Produce a linear velocity using sigmoid
	lin_v = 30*pow(2.0,(f_rng_avg/100)-5)/(pow(2.0,(f_rng_avg/100)-5) + 4.0);

	// Calculate the average of the 5 right beams about -45 degrees
	for(uint16_t r_ndx = 312; r_ndx < 317; r_ndx++){
		
		if(range[r_ndx] > MIN_RANGE){
			r_rng_avg += (float)range[r_ndx];
			r_cntr++;
		}
	}
	if(r_cntr != 0){
		r_rng_avg = r_rng_avg/(float)r_cntr;
	}
//	rprintf("Right Range: ");
//	rprintfFloat(5, r_rng_avg);
//	rprintfCRLF();

	ang_v = -0.3*(r_rng_avg-230)/sqrt(1 + square(r_rng_avg-230));

	send_frame(lin_v,ang_v);
	rprintf("400, ");
	rprintfFloat(5,ang_v);
	rprintfCRLF();
	rprintf("500, ");
	rprintfFloat(5,lin_v);
	rprintfCRLF();

}

int main(void)
{
	uint8_t audio_start = UNSET;
	unsigned short audio_value[1000];
	uint8_t audio_count = 0;

	uint16_t ranges[360];
	double ang_vel = 0;
	double lin_vel = 0;
	init_LDS_buffer();
	prvSetupHardware();
	rprintf("Starting program.\n");
	
	while(!audio_start){
		if(a2dConvert10bit(AUDIO_PIN) > AUDIO_THRESH){
			for(uint16_t s_ndx = 0; s_ndx < 1000; s_ndx++){
				audio_value[s_ndx] = a2dConvert10bit(AUDIO_PIN);
				//if(a2dConvert10bit(AUDIO_PIN) > AUDIO_THRESH){
					audio_count++;
				//}
				//rprintf("%d\n",a2dConvert10bit(AUDIO_PIN));
				delay_us(10);	// Sample every 10 us
			}
			for(uint16_t s_ndx = 0; s_ndx < 1000; s_ndx++){
				rprintf("%d\n",audio_value[s_ndx]);
			}
			if(audio_count > PEAKS_IN_10ms){
				audio_start = SET;
			}
			else{
				audio_count = 0;
			}
			
		}
		delay_us(10);	// Sample every 10 us
	}
	
	rprintf("START!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	while(1){
		// Print ranges
		//rprintf("Scan start\n");
/**/
		get_range_scan(ranges);
		// Print ranges //
		/*
		for(uint16_t r_ndx = 0; r_ndx < 360; r_ndx++){
			rprintf("%d,%d",r_ndx,ranges[r_ndx]);
			//uart2SendByte(ranges[r_ndx]);
			rprintfCRLF();
			//uart2SendByte('\n');
		}
		*/
		//rprintf("\nScan end\n\n");
		
		//right_wall(ranges);
		/*
		send_frame(18,0);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		send_frame(0,0.25);
		delay_ms(500);
		send_frame(18,0);
		delay_ms(1000);
		send_frame(0,0.5);
		delay_ms(1000);
		send_frame(30,0);
		delay_ms(1000);
		send_frame(12,0.125);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
		*/
		navupdate(ranges, &lin_vel, &ang_vel);
		send_frame(lin_vel,ang_vel);
		delay_ms(10);

		//BRIAN==========================================================		
		//. Uptake [distance, velocity][n = 1:360]
		/*
		for( int i=0; i<360; i++ ) {
			addFrame(_FRMIDX2THETA(i),ranges[i]);
			send_frame( (float)_GETVELOC, (float)_GETOMEGA );
				
			solutionUpdater();
		}
		*/
		/*
		fow		=0;
		right	=0;
		for( int i = 0; i < 8; i++ ) {
			fow   += ranges[i]  + ranges[360-i];
			right += ranges[90+2*i] + ranges[90-2*i];
		}
		fow = (fow - 15000)/400;
		right = 30*((right - 8000)/8000+1);
		send_frame( fow, right );

		rprintfFloat( 6, fow   ); rprintfChar( '\t' );
		rprintfFloat( 6, right  ); rprintfChar( '\n' );
		//===============================================================		
		*/
		
	}

	rprintf("Program terminated.");
	
	return 0;
}
