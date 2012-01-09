#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>

#ifndef TIMER640_H
#define TIMER640_H

/* These are used to set the prescaler for timers 0,1,3,4,5 */
#define TIMER_CLK_STOP    0x00 ///< Timer Stopped
#define TIMER_CLK_1       0x01 ///< Timer clocked at F_CPU
#define TIMER_CLK_8       0x02 ///< Timer clocked at F_CPU/8
#define TIMER_CLK_64      0x03 ///< Timer clocked at F_CPU/64
#define TIMER_CLK_256     0x04 ///< Timer clocked at F_CPU/256
#define TIMER_CLK_1024    0x05 ///< Timer clocked at F_CPU/1024
#define TIMER_CLK_T_FALL  0x06 ///< Timer clocked at T falling edge
#define TIMER_CLK_T_RISE  0x07 ///< Timer clocked at T rising edge

/* These are used to set the prescaler for timer 2 only */
#define TIMER2_CLK_STOP   0x00 ///< Timer Stopped
#define TIMER2_CLK_1      0x01 ///< Timer clocked at F_CPU
#define TIMER2_CLK_8      0x02 ///< Timer clocked at F_CPU/8
#define TIMER2_CLK_32     0x03 ///< Timer clocked at F_CPU/32
#define TIMER2_CLK_64     0x04 ///< Timer clocked at F_CPU/64
#define TIMER2_CLK_128    0x05 ///< Timer clocked at F_CPU/128
#define TIMER2_CLK_256    0x06 ///< Timer clocked at F_CPU/256
#define TIMER2_CLK_1024   0x07 ///< Timer clocked at F_CPU/1024

#define TIMER0_MAX      255
#define TIMER1_MAX      65535
#define TIMER2_MAX      255
#define TIMER3_MAX      65535
#define TIMER4_MAX      65535
#define TIMER5_MAX      65535

typedef enum TimerInterrupt { 
  TIMER0_COMPA_interrupt = 0, // Timer/Counter0 Compare Match A
  TIMER0_COMPB_interrupt,     // Timer/Counter0 Compare Match B
  TIMER0_OVF_interrupt,       // Timer/Counter0 Overflow
  TIMER1_CAPT_interrupt,      // Timer/Counter1 Capture Event
  TIMER1_COMPA_interrupt,     // Timer/Counter1 Compare Match A
  TIMER1_COMPB_interrupt,     // Timer/Counter1 Compare Match B
  TIMER1_COMPC_interrupt,     // Timer/Counter1 Compare Match C
  TIMER1_OVF_interrupt,       // Timer/Counter1 Overflow
  TIMER2_COMPA_interrupt,     // Timer/Counter2 Compare Match A
  TIMER2_COMPB_interrupt,     // Timer/Counter2 Compare Match B
  TIMER2_OVF_interrupt,       // Timer/Counter2 Overflow
  TIMER3_CAPT_interrupt,      // Timer/Counter3 Capture Event
  TIMER3_COMPA_interrupt,     // Timer/Counter3 Compare Match A
  TIMER3_COMPB_interrupt,     // Timer/Counter3 Compare Match B
  TIMER3_COMPC_interrupt,     // Timer/Counter3 Compare Match C
  TIMER3_OVF_interrupt,       // Timer/Counter3 Overflow
  TIMER4_CAPT_interrupt,      // Timer/Counter4 Capture Event
  TIMER4_COMPA_interrupt,     // Timer/Counter4 Compare Match A
  TIMER4_COMPB_interrupt,     // Timer/Counter4 Compare Match B
  TIMER4_COMPC_interrupt,     // Timer/Counter4 Compare Match C
  TIMER4_OVF_interrupt,       // Timer/Counter4 Overflow
  TIMER5_CAPT_interrupt,      // Timer/Counter5 Capture Event
  TIMER5_COMPA_interrupt,     // Timer/Counter5 Compare Match A
  TIMER5_COMPB_interrupt,     // Timer/Counter5 Compare Match B
  TIMER5_COMPC_interrupt,     // Timer/Counter5 Compare Match C
  TIMER5_OVF_interrupt,       // Timer/Counter5 Overflow
  TIMER_MAX_ENUM} TimerInterrupt_t;

void delay_us(unsigned short time_us);
#define delay_ms sleep
void sleep(uint16_t time_ms);

uint16_t prescaler_hex_to_value(uint8_t hex);
uint16_t prescaler_hex_to_value_for_timer2(uint8_t hex);

uint16_t get_timer0_prescaler(void);
uint16_t get_timer1_prescaler(void);
uint16_t get_timer2_prescaler(void);
uint16_t get_timer3_prescaler(void);
uint16_t get_timer4_prescaler(void);
uint16_t get_timer5_prescaler(void);

/* 8-bit sync timer 
 */
void init_timer0(const uint8_t prescaler);

/* 16-bit sync timer
 */
void init_timer1(const uint8_t prescaler);

/* 8-bit async timer 
 */
void init_timer2(const uint8_t prescaler);

/* 16-bit sync timer
 */
void init_timer3(const uint8_t prescaler);

/* 16-bit sync timer
 */
void init_timer4(const uint8_t prescaler);

/* 16-bit sync timer
 */
void init_timer5(const uint8_t prescaler);

void reset_timer0(void);
void reset_timer1(void);
void reset_timer2(void);
void reset_timer3(void);
void reset_timer4(void);
void reset_timer5(void);

const uint8_t get_timer0_counter(void);
const uint16_t get_timer1_counter(void);
const uint8_t get_timer2_counter(void);
const uint16_t get_timer3_counter(void);
const uint16_t get_timer4_counter(void);
const uint16_t get_timer5_counter(void);

const uint32_t get_timer0_overflow(void);
const uint32_t get_timer1_overflow(void);
const uint32_t get_timer2_overflow(void);
const uint32_t get_timer3_overflow(void);
const uint32_t get_timer4_overflow(void);
const uint32_t get_timer5_overflow(void);

void timer_attach(TimerInterrupt_t interrupt, void (*user_func)(void) );
void timer_detach(TimerInterrupt_t interrupt);

#endif
