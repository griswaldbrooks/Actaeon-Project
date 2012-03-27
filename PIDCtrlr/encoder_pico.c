#include <encoder_pico.h>

s32 wheel_left_tick_counter  = 0;
s32 wheel_right_tick_counter = 0;

u32 get_left_count(void){
	return wheel_left_tick_counter;
}

u32 get_right_count(void){
	return wheel_right_tick_counter;
}

SIGNAL(PCINT2_vect){
	// Only enabled for Digital Pin 7, Encoder Left
	static u08 pin_level_past = 0;
	static u08 pin_level_current;

	// Grab current pin state
	pin_level_current = bit_is_set(PIND,7);

	if(pin_level_current > pin_level_past){		// Positive edge trigger
		if(bit_is_set(PINB,0)){					// Robot left wheel reverse
			wheel_left_tick_counter--;
		}
		else{									// Robot left wheel forward
			wheel_left_tick_counter++;
		}
	}
	else{										// Negative edge trigger
		if(bit_is_set(PINB,0)){
			wheel_left_tick_counter++;			// Robot left wheel forward
		}
		else{
			wheel_left_tick_counter--;			// Robot left wheel reverse
		}
	}
	pin_level_past = pin_level_current;
	
}
SIGNAL(PCINT0_vect){
	// Enabled for Digital Pin 8, Encoder Left
	// and Digital Pin 12 and 13, Encoder Right

	static u08 PINB0_past = 0;
	static u08 PINB0_current;
	static u08 PINB4_past = 0;
	static u08 PINB4_current;
	static u08 PINB5_past = 0;
	static u08 PINB5_current;

	// Check pin statuses and determine which pin cause the interrupt
	PINB0_current = bit_is_set(PINB,0);
	PINB4_current = bit_is_set(PINB,4);
	PINB5_current = bit_is_set(PINB,5);

	if(PINB0_current != PINB0_past){
		if(PINB0_current > PINB0_past){		// Positive edge trigger
			if(bit_is_set(PIND,7)){					// Robot left wheel reverse
				wheel_left_tick_counter++;
			}
			else{									// Robot left wheel forward
				wheel_left_tick_counter--;
			}
		}
		else{										// Negative edge trigger
			if(bit_is_set(PIND,7)){
				wheel_left_tick_counter--;			// Robot left wheel forward
			}
			else{
				wheel_left_tick_counter++;			// Robot left wheel reverse
			}
		}
		PINB0_past = PINB0_current;
		
	}
	else if(PINB4_current != PINB4_past){
		if(PINB4_current > PINB4_past){				// Positive edge trigger
			if(bit_is_set(PINB,5)){					// Robot right wheel forward
				wheel_right_tick_counter++;
			}
			else{									// Robot right wheel reverse
				wheel_right_tick_counter--;
			}
		}
		else{										// Negative edge trigger
			if(bit_is_set(PINB,5)){
				wheel_right_tick_counter--;			// Robot right wheel reverse
			}
			else{
				wheel_right_tick_counter++;			// Robot right wheel forward
			}
		}
		PINB4_past = PINB4_current;
		//rprintf("%d\n",wheel_right_tick_counter);
	}
	else{
		if(PINB5_current > PINB5_past){				// Positive edge trigger
			if(bit_is_set(PINB,4)){					// Robot right wheel reverse
				wheel_right_tick_counter--;
			}
			else{									// Robot right wheel forward
				wheel_right_tick_counter++;
			}
		}
		else{										// Negative edge trigger
			if(bit_is_set(PINB,4)){
				wheel_right_tick_counter++;			// Robot right wheel forward
			}
			else{
				wheel_right_tick_counter--;			// Robot right wheel reverse
			}
		}
		PINB5_past = PINB5_current;
		//rprintf("%d\n",wheel_right_tick_counter);
	}
		
}


ISR(BADISR_vect){
	
}
