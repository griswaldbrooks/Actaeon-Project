/*!	\file	obstacle_avoidance.h
 *	\brief	Obstacle avoidance library.
 *
 *			GENERAL:
 *			Library is intended for robots utilizing range finder mechanisms,
 *			such as laser or ir range finders.
 *
 *			USAGE NOTES:
 *			When frames are added utilizing addFrame, the solutionUpdate function
 *			must be called, in respective sequence, so that outputs can be generated
 *			with respect to expected input frame count. 
 *
 */
 
#ifndef 	obstacle_avoidance_h
#define 	obstacle_avoidance_h

#include 	<rprintf.h>

/*-VARIABLES----------------------------------------------------------------------------*/
/*!	\var 	unsigned int _framecount
 *	\brief	Current number of inputted frames
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	unsigned int _veloccount
 *	\brief	Current number of frames considered for velocity calculations
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	unsigned int _framecount
 *	\brief	Current number of inputted frames
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	unsigned int _scopecount
 *	\brief	Current number of frames in scope for aperature detection
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_currX
 *	\brief	Cartesian representation of current in-scope point; x-component
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_currY
 *	\brief	Cartesian representation of current in-scope point; y-component
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_lastX
 *	\brief	Cartesian representation of previous in-scope point; x-component
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_lastY
 *	\brief	Cartesian representation of previous in-scope point; y-component
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_wieghtedsum
 *	\brief	Sum of potential .* theta
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_velocitysum
 *	\brief	Sum of range components for velocity caclulation
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_potentialsum
 *	\brief	Sum of potential field values
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_omegasolution
 *	\brief	Omega output storage
 */
/*--------------------------------------------------------------------------------------*/
/*!	\var 	double	_velocsolution
 *	\brief	Velocity output storage
 */
/*--------------------------------------------------------------------------------------*/
/*!	\fn 	double	_velocsolution
 *	\brief	Velocity output storage
 */
/*-FUNCTIONS----------------------------------------------------------------------------*/
/*!	\fn 	addFrame( double theta, double radius )
 *	\brief	Input new frame.
 *			Handles counter resets.
 *	\param	[in]	theta	range-finder angle {0<tht<2*pi}
 *	\param  [in]	radius	range-finder radius {0<rad<inf}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\fn 	solutionUpdater(void)
 *	\brief	Handles solution updating.
 *			Dependent on frame counter. Muts be called after addFrame function to 
 *			monitor number of frame inputs, and further, to known when to execute
 *			solution generation.
 */
/*--------------------------------------------------------------------------------------*/

unsigned int	_framecount 	= 0;
unsigned int	_veloccount		= 0;
unsigned int	_scopecount		= 0;
double			_currX 			= 0;
double			_currY			= 0;
double			_lastX 			= 0;
double			_lastY			= 0;
double			_velocitysum	= 0;
double 			_potentialsum 	= 0;
double			_wieghtedsum	= 0;
double			_omegasolution 	= 0;
double			_velocsolution 	= 0;

#include 	<obstacle_avoidance_defines.h>

void addFrame( double theta, double radius );
void solutionUpdater(void);


// Source moved to header for odd inclusion error
//=============================================================
// 1:51 PM 
// 3-19-2012
// Brian Cairl
//=============================================================


/*!	\fn 	addFrame( double theta, double radius )
 */ 
void addFrame( double theta, double radius ) {
	// ============= VARIABLE RESET HANDLE SECTION ============ //
	if( _framecount > (frames_NEXPECTED) ) {
		_framecount 	= 0;
		_veloccount		= 0;
		_scopecount		= 0;
		_lastX 			= 0;
		_lastY			= 0;
		_velocitysum	= 0;
		_potentialsum 	= 0;
		_wieghtedsum	= 0;
		_omegasolution	= 0;
	}
	// ============ POTENTIAL FIELD HANDLE SECTION ============ //
	double ith_potential, ith_angle;
	_framecount++;
	if( radius > 5 ) {
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
				rprintfFloat( 4,_omegasolution );
			}
		}
	}
}



/*!	\fn 	solutionUpdater(void)
 */
void solutionUpdater(void) {
	if( _framecount == frames_NEXPECTED ) {
	// ========== VELOCITY CALCULATION HANDLE SECTION ========= //
		_velocsolution  = (_velocitysum/_veloccount)*veloc_LINEARCOEFF;
		rprintfFloat( 3, _veloccount );
	// ============== BAD READING HANDLE SECTION ============= //	
		if( _veloccount < 5 ) {
			_velocsolution = -10;
		}
	// ============ OMEGA CALCULATION HANDLE SECTION ========== //
		_omegasolution += (double)(_wieghtedsum/_potentialsum)*omega_LINEARCOEFF;	

		rprintfStr( "==============================\n" );
		rprintfFloat( 4, _omegasolution ); 	rprintfChar( '\t' );
		rprintfFloat( 4, _velocsolution ); 	rprintfChar( '\n' );
		rprintfStr( "------------------------------\n" );		
		rprintfFloat( 4, _velocitysum ); 	rprintfChar( '\t' );
		rprintfFloat( 4, _wieghtedsum ); 	rprintfChar( '\n' );
		rprintfStr( "==============================\n" );

	}
} 

#endif
