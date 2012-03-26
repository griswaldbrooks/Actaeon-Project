#ifndef GLOBAL_H
#define GLOBAL_H

// global AVRLIB defines
#include <avrlibdefs.h>
// global AVRLIB types definitions
#include <avrlibtypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

//#define F_CPU        16000000               		// 16MHz processor
#define CYCLES_PER_US ((F_CPU+500000)/1000000) 	// cpu cycles per microsecond

#endif
