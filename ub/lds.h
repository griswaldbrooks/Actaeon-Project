#ifndef	LDS_H
#define LDS_H

//#include <SoR_Utils.h>
#include <stdint.h>
#include <stdlib.h>
#include <timer640.h>
#include <rprintf.h>

#define SET		0x01;
#define UNSET	0x00;
#define TRUE	0x01;
#define FALSE	0x00;

#define BUFFER_SIZE		256

typedef struct {
	uint8_t index;
	uint16_t motor_speed;
	uint8_t invalid[4];
	uint8_t strength_warning[4];
	uint16_t distance[4];
	uint16_t intensity[4];
	uint16_t checksum;

} LDS_FRAME;

void init_LDS_buffer();

void LDSRcv(unsigned char c);

uint8_t read_LDS();

uint8_t error_checker(const uint16_t chk_data[], uint16_t checksum);

uint8_t parse_frame(LDS_FRAME *frame);

void print_frame(LDS_FRAME frame);

void grab_frames(const LDS_FRAME *frame, LDS_FRAME ldsf[]);

void conv_FrametoDist(const LDS_FRAME ldsf[], uint16_t ranges[]);

void get_range_scan(uint16_t ranges[]);



#endif
