#ifndef ENCODER_PICO_H
#define ENCODER_PICO_H

#include <global.h>
#include <rprintf.h>

#define PORT_ON( port_letter, number )			port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )			port_letter &= ~(1<<number)

u32 get_left_count(void);
u32 get_right_count(void);

#endif
