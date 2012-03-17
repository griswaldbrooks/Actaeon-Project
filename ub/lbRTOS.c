#include <SoR_Utils.h>
#include <lds.h>

#define SET		0x01;
#define UNSET	0x00;

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

	// Send linear velocity
	uart0SendByte(fltuint8_velocity.arr_vel[0]);
	uart0SendByte(fltuint8_velocity.arr_vel[1]);
	uart0SendByte(fltuint8_velocity.arr_vel[2]);
	uart0SendByte(fltuint8_velocity.arr_vel[3]);

	// Send angular velocity
	uart0SendByte(fltuint8_omega.arr_ome[0]);
	uart0SendByte(fltuint8_omega.arr_ome[1]);
	uart0SendByte(fltuint8_omega.arr_ome[2]);
	uart0SendByte(fltuint8_omega.arr_ome[3]);

	// Send checksum
	uint8_t chk = fltuint8_velocity.arr_vel[0] + fltuint8_velocity.arr_vel[1] + fltuint8_velocity.arr_vel[2]
	+ fltuint8_velocity.arr_vel[3] + fltuint8_omega.arr_ome[0] + fltuint8_omega.arr_ome[1] 
	+ fltuint8_omega.arr_ome[2] + fltuint8_omega.arr_ome[3];

	uart0SendByte(chk);
}


int main(void)
{
	
	uint16_t ranges[360];
	init_LDS_buffer();
	prvSetupHardware();
	rprintf("Starting program.\n");

	while(1){
		// Print ranges
		//rprintf("Scan start\n");
		get_range_scan(ranges);
		/*
		for(uint16_t r_ndx = 0; r_ndx < 360; r_ndx++){
			rprintf("%d",ranges[r_ndx]);
			rprintfCRLF();
		}
		rprintf("\nScan end\n\n");
		*/
		
		delay_ms(1000);

	}

	rprintf("Program terminated.");
	
	return 0;
}
