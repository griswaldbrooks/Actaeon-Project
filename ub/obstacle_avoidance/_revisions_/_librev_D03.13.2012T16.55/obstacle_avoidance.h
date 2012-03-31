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

#include 	obstacle_avoidance_defines.h

void addFrame( double theta, double radius );
void solutionUpdater(void);

#endif