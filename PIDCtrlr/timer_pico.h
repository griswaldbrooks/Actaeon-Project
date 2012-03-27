#ifndef TIMER_PICO_H
#define TIMER_PICO_H

#include <global.h>

/* These are used to set the prescaler for timers 0,1 */
#define TIMER_CLK_STOP    0x00 ///< Timer Stopped
#define TIMER_CLK_1       0x01 ///< Timer clocked at F_CPU
#define TIMER_CLK_8       0x02 ///< Timer clocked at F_CPU/8
#define TIMER_CLK_64      0x03 ///< Timer clocked at F_CPU/64
#define TIMER_CLK_256     0x04 ///< Timer clocked at F_CPU/256
#define TIMER_CLK_1024    0x05 ///< Timer clocked at F_CPU/1024
#define TIMER_CLK_T_FALL  0x06 ///< Timer clocked at T falling edge
#define TIMER_CLK_T_RISE  0x07 ///< Timer clocked at T rising edge

volatile uint32_t timer0_ovrflow_cnt;
volatile uint32_t timer1_ovrflow_cnt;

void reset_timer0(void);
void reset_timer1(void);
void init_timer0(const u08 prescaler);
void init_timer1(const u08 prescaler);
const uint32_t get_timer0_overflow(void);
void delay_us(unsigned short time_us) ;

#endif
