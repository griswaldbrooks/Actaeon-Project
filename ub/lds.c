#include <lds.h>

uint8_t lds_buffer[BUFFER_SIZE];
uint8_t *lds_buffer_write_ndx = NULL;
uint8_t *lds_buffer_read_ndx = NULL;

void init_LDS_buffer(){
	lds_buffer_write_ndx = lds_buffer_read_ndx = lds_buffer;
}

void LDSRcv(unsigned char c){
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
	uint16_t r_ndx;
	for(uint8_t f_ndx = 0; f_ndx < 90; f_ndx++){
		// Maps frame indicies to range indices
		r_ndx = 4*ldsf[f_ndx].index - 640;
		ranges[r_ndx]     = (ldsf[f_ndx].distance[0]);
		ranges[r_ndx + 1] = (ldsf[f_ndx].distance[1]);
		ranges[r_ndx + 2] = (ldsf[f_ndx].distance[2]);
		ranges[r_ndx + 3] = (ldsf[f_ndx].distance[3]);
	}
}

void get_range_scan(uint16_t ranges[]){
	// ranges must be of length 360
	uint8_t first_frame = UNSET;
	LDS_FRAME frame;
	LDS_FRAME ldsf[90];
	// Grab frames
	while(!first_frame){
		if(parse_frame(&frame)){
			//rprintf("Got frame.\n");
			// If you get the first frame, start reading into the frame buffer
			if(frame.index == 0xA0){
				first_frame = SET;
				grab_frames(&frame,ldsf);
				//rprintf("Frames grabbed.\n");
				// Convert frames into distances
				conv_FrametoDist(ldsf,ranges);
				//rprintf("Frames Converted.\n");
				// Print ranges
				/*
				rprintf("Scan start\n");
				for(uint16_t r_ndx = 0; r_ndx < 360; r_ndx++){
					rprintf("%d",ranges[r_ndx]);
					rprintfCRLF();
				}
				rprintf("\nScan end\n\n");
				*/
			}
		}
	}
}

