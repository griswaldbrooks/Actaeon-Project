/*!	\file	wall_follow.h
 *	\brief	Wall following algorithm set.
 */
 
#ifndef			wall_follow_h
#define			wall_follow_h

#include		<math.h>


/* TOGGLES: */
#define			SN_LEVELING					// 'Leveling' routine toggle	
#define			SN_FIXEDCONST				// Fixed shifting routine toggle
#define			SN_VELOCITY					// Velocity calculation toggle

/* COSTANTS: */
#define			NSCANS				360 	// number of input scans
#define			WIDTH_APERATURE		40		// width of hallway aperature
#define			WIDTH_ROBOT			20		// robot width
#define			SECTOR_CORRECT		(pi/36)	// Fixed shifting routine constant
#define			GENERAL_SCALING		(1E4)	// General divide-out scaling
#define			VELOCITY_SCALING	(10)	// Arb. velcoity scaling
#define			LEVELING_LAYERS		2 		// < max 4 >

#define			HEADINGZERO(n)		(n<=180)?(n):(n-360)
#define			SECTOR_BOUNDS		{0,45,90,135,180,225,270,315,360} 


/* PULL-OFF VARIABLES: */
double			_omega;
double			_veloc;		


double			sector_sum[8];	
unsigned int	sector_bounds[9];		


/* INITIALIZATION FN */
void sector_init(void) {
	unsigned int _tmp[9]=SECTOR_BOUNDS;
	for(int i=0; i<9; i++ ) {
		sector_bounds[i] = _tmp[i];
	}
}




/* MAIN ROUTINE CALL FN */
void sector_routine( unsigned int ranges[NSCANS] ) {
	_omega = 0;
	_veloc = 0;

	
	//-> Fill sectors
	for( int i=0; i<8; i++ ) {
		sector_sum[i] = 0;
		for( int n=sector_bounds[i]; n<sector_bounds[i+1]; n++ ) {
			sector_sum[i] += (double)(ranges[n]);
		}
	}
	
	
	#ifdef SN_LEVELING
	//-> Compute 'leveling' correction
		for( int i=0; i<LEVELING_LAYERS; i++ ) {
			_omega += (sector_sum[7-i]-sector_sum[i])/(LEVELING_LAYERS*GENERAL_SCALING);
		}
	#endif
	
	
	#ifdef SN_FIXEDCONST
	//-> Fixed constant correction
		_omega -= (double)((sector_sum[0]-(WIDTH_APERATURE-WIDTH_ROBOT)/2)+(sector_sum[1]-2*WIDTH_ROBOT))*(SECTOR_CORRECT/(2*GENERAL_SCALING));
		_omega += (double)((sector_sum[7]-(WIDTH_APERATURE-WIDTH_ROBOT)/2)+(sector_sum[6]-2*WIDTH_ROBOT))*(SECTOR_CORRECT/(2*GENERAL_SCALING));
	#endif
	
	
	#ifdef SN_VELOCITY
	//-> Velocity Calculation
		_veloc += (double)(sector_sum[0]+sector_sum[7]-WIDTH_ROBOT)/(180+90*WIDTH_ROBOT))*VELOCITY_SCALING;
	#else
		_veloc += VELOCITY_SCALING;
	#endif
	
}




#endif