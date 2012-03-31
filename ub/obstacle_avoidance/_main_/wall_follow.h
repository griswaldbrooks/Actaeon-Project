/*!	\file	wall_follow.h
 *	\brief	Wall following algorithm set.
 */
 
#ifndef			wall_follow_h
#define			wall_follow_h

#include		<math.h>

#ifdef			ARDUINO_PLATFORM
#include		<wprogram.h>
#endif


#define			nScans		360
#define			rel2head(a)	(a<180)?(a*M_PI/180):((a*M_PI/180)-2*M_PI)
#define			robotwidth 	20
#define			scaling		1E4
#define			blinder		40

//double			_omega;
//double			_veloc;


void navupdate( unsigned int ranges[nScans], double* _veloc, double* _omega ) {
	//.RESET PICK-OFFS
	*_omega = 0; 
	*_veloc = 0;
	
	double	total_pot        ,
			effective_pot 	 ;
	
	//.INITIAL RANGE-SUM
	for( int i=0; i<nScans; i++ ) {
		total_pot+=ranges[i];
	}
	for( int i=0; i<nScans; i++ ) {
		//.POTENTIAL FIELD AREA
		effective_pot = (ranges[i]-robotwidth)/total_pot;
		*_omega += rel2head(i)/effective_pot;
		//.SIMPLE L-R WALL FOLLOWING
		if( (i>44) && (i<135-blinder)){
			*_omega += (ranges[i]-ranges[360-i-1])/scaling;
		}
		//.VELOCITY RECALCULATION
		if( i>44 ){
			*_veloc += ((ranges[i]      -robotwidth)
					  -(ranges[360-i-1]-robotwidth))/scaling;
		}
	}

}




#endif
