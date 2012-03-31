/*!	\file
 *	\brief
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
	ith_angle 		= _REL2HEADING(theta);
	ith_potential	= _POTENTIAL_FIELD(ith_angle,radius);
	_potentialsum  += ith_potential;
	_wieghtedsum   += ith_potential*ith_angle;
	// ========== VELOCITY CACLULATION HANDLE SECTION ========= //
	if( fabs(ith_angle) < veloc_ANGLESCOPE ) {
		_veloccount++;
		_velocitysum += _POTENTIAL_rfrad(radius);
	}
	// =========== IN-SCOPE-SORTING HANDLE SECTION =========== //
	unsigned int scope_update = 0;
	if( radius < omega_RADSCOPE ) {
		_scopecount++;
		_currX = radius*sin(ith_angle);
		_currY = radius*cos(ith_angle);
		scope_update = 1;
	}
	// ========== APERATURE DETECTION HANDLE SECTION ========== //
	if( _scopecount > 0 && scope_update ) {
	
	
	
	}

}