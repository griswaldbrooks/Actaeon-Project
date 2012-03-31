/*!	\file
 *	\brief
 */
 
#ifndef 	obstacle_avoidance_h
#define 	obstacle_avoidance_h

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