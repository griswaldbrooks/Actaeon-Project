#include <pwm_pico.h>

// Initialize PWM, set timer resolution

void PWM_Init_timer1_Pin9(u08 res){
	// enable timer1 as 8,9,10bit PWM
	sbi(TCCR1B,WGM12);
	cbi(TCCR1B,WGM13);
	if(res == 9)
	{	// 9bit mode
		sbi(TCCR1A,WGM11);
		cbi(TCCR1A,WGM10);
	}
	else if(res == 10 )
	{	// 10bit mode
		sbi(TCCR1A,WGM11);
		sbi(TCCR1A,WGM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR1A,WGM11);
		sbi(TCCR1A,WGM10);
	}
	// clear output compare values
	OCR1A = 0;
}
void PWM_Init_timer1_Pin10(u08 res){
	// enable timer1 as 8,9,10bit PWM
	cbi(TCCR1B,WGM12);
	cbi(TCCR1B,WGM13);
	if(res == 9)
	{	// 9bit mode
		sbi(TCCR1A,WGM11);
		cbi(TCCR1A,WGM10);
	}
	else if(res == 10 )
	{	// 10bit mode
		sbi(TCCR1A,WGM11);
		sbi(TCCR1A,WGM10);
	}
	else
	{	// default 8bit mode
		cbi(TCCR1A,WGM11);
		sbi(TCCR1A,WGM10);
	}
	// clear output compare values
	OCR1B = 0;
}

void timer1PWMInitICR(u16 topcount)
{
	u08 sreg;
	// set PWM mode with ICR top-count
	cbi(TCCR1A,WGM10);
	sbi(TCCR1A,WGM11);
	sbi(TCCR1B,WGM12);
	sbi(TCCR1B,WGM13);
	
	// Save global interrupt flag
	sreg = SREG;
	// Disable interrupts
	cli();
	// set top count value
	ICR1 = topcount;
	// Restore interrupts
	SREG = sreg;
	
	// clear output compare values
	OCR1A = 0;
	OCR1B = 0;
}

// Stop PWM
void PWM_timer0_Off_Pin5(void){
	cbi(TCCR0A,COM0B1);
	cbi(TCCR0A,COM0B0);
}
void PWM_timer0_Off_Pin6(void){
	cbi(TCCR0A,COM0A1);
	cbi(TCCR0A,COM0A0);
}
void PWM_timer1_Off_Pin9(void){
	cbi(TCCR1A,COM1B1);
	cbi(TCCR1A,COM1B0);
}
void PWM_timer1_Off_Pin10(void){
	cbi(TCCR1A,COM1A1);
	cbi(TCCR1A,COM1A0);
}

// Start PWM
void PWM_timer0_On_Pin5(void){
	sbi(TCCR0A,COM0B1);
	cbi(TCCR0A,COM0B0);
}
void PWM_timer0_On_Pin6(void){
	sbi(TCCR0A,COM0A1);
	cbi(TCCR0A,COM0A0);
}
void PWM_timer1_On_Pin9(void){
	sbi(TCCR1A,COM1B1);
	cbi(TCCR1A,COM1B0);
}
void PWM_timer1_On_Pin10(void){
	sbi(TCCR1A,COM1A1);
	cbi(TCCR1A,COM1A0);
}
// Set duty
void PWM_timer0_Set_Pin5(u16 duty){
	OCR0B = duty;
}
void PWM_timer0_Set_Pin6(u16 duty){
	OCR0A = duty;
}
void PWM_timer1_Set_Pin9(u16 duty){
	OCR1A = duty;
}
void PWM_timer1_Set_Pin10(u16 duty){
	OCR1B = duty;
}
