/*!	\file	obstacle_avoidance.c
 *	\brief	Source for obstacle avoidance library.
 */
 
#include	"obstacle_avoidance.h"
//#include	<obstacle_avoidance_defines.h>

/*!	\fn 	addFrame( double theta, double radius )
 */ 
void addFrame( double theta, double radius ) {
	// ============= VARIABLE RESET HANDLE SECTION ============ //
	if( _framecount >= frames_NEXPECTED ) {
		_framecount 	= 0;
		_veloccount		= 0;
		_scopecount		= 0;
		_lastX 			= 0;
		_lastY			= 0;
		_velocitysum	= 0;
		_potentialsum 	= 0;
	}
	// ============ POTENTIAL FIELD HANDLE SECTION ============ //
	double ith_potential, ith_angle;
	_framecount++;
	ith_angle 		= _REL2HEADING(theta);
	ith_potential	= _POTENTIAL_FIELD(ith_angle,radius);
	_potentialsum  += ith_potential;
	_wieghtedsum   += ith_potential*ith_angle;
	// ========== VELOCITY CALCULATION HANDLE SECTION ========= //
	if( fabs(ith_angle) < veloc_ANGLESCOPE ) {
		_veloccount++;
		_velocitysum += _POTENTIAL_rfrad(radius);
	}
	// =========== IN-SCOPE-SORTING HANDLE SECTION =========== //
	unsigned int scope_update = 0;
	if( radius < omega_RADSCOPE ) {
		_scopecount++;
		_lastX = _currX; 
		_lastY = _currY;
		_currX = radius*sin(ith_angle);
		_currY = radius*cos(ith_angle);
		scope_update = 1;
	}
	// ========== APERATURE DETECTION HANDLE SECTION ========== //
	if( (_scopecount > 0) && scope_update ) {
		double magnitude;
		magnitude = sqrt(pow(_lastX-_currX,2)+pow(_lastY-_currY,2));
		if( _LOGIT(magnitude/apdet_APWIDTH) > fndef_LOGITCUTOFF ) {
			_omegasolution += atan2((_lastX+_currX)/2,(_lastY+_currY)/2);
		}
	}
	rprintFloat( 2, _framecount );
}



/*!	\fn 	solutionUpdater(void)
 */

void solutionUpdater(void) {
	if( _framecount >= frames_NEXPECTED ) {
	rprintStr( "SOLUTION UPDATED" );
	// ========== VELOCITY CALCULATION HANDLE SECTION ========= //
		_velocsolution  = _velocitysum/_veloccount*veloc_LINEARCOEFF;
	
	// ============ OMEGA CALCULATION HANDLE SECTION ========== //
		_omegasolution += _wieghtedsum/_potentialsum*omega_LINEARCOEFF;
	}
} 