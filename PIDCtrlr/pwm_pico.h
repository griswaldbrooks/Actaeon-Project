#ifndef PWM_PICO_H
#define PWM_PICO_H

#include <global.h>

// Initialize PWM, set timer resolution
void PWM_Init_timer0_Pin5(u08 res);
void PWM_Init_timer0_Pin6(u08 res);
void PWM_Init_timer1_Pin9(u08 res);
void PWM_Init_timer1_Pin10(u08 res);

// Set top-count value for for Input Capture Register
void timer1PWMInitICR(u16 topcount);

// Stop PWM
void PWM_timer0_Off_Pin5(void);
void PWM_timer0_Off_Pin6(void);
void PWM_timer1_Off_Pin9(void);
void PWM_timer1_Off_Pin10(void);

// Start PWM
void PWM_timer0_On_Pin5(void);
void PWM_timer0_On_Pin6(void);
void PWM_timer1_On_Pin9(void);
void PWM_timer1_On_Pin10(void);

// Set duty
void PWM_timer0_Set_Pin5(u16 duty);
void PWM_timer0_Set_Pin6(u16 duty);
void PWM_timer1_Set_Pin9(u16 duty);
void PWM_timer1_Set_Pin10(u16 duty);

#endif
