/*!	\file	obstacle_avoidance_defines.h
 *	\brief 	Definition file for obstacle avoidance library.
 *			Includes all settable constants and functional macros.
 */
 
#ifndef  	obstacle_avoidance_defines_h
#define  	obstacle_avoidance_defines_h

#include 	<math.h>

/*-CONSTANT-----------------------------------------------------------------------------*/
/*!	\def 	frames_NEXPECTED
 *	\brief	Number of expected frames expected in one range-finder sweep
 *			A frame is considered to be an input of the format {radius,theta}
 *	\see 	addFrame
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	veloc_LINEARCOEFF
 *	\brief	Linear coefficient applied to velocity output solution
 *			Can be utilized to make proportional adjustments to output magnitudes.
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	veloc_ANGLESCOPE
 *	\brief	Angular region considered during velocity calculation
 *			Represents sweep region from { -(SCP)<THETA<+(SCP) }
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	omega_LINEARCOEFF
 *	\brief	Linear coefficient applied to omega output solution
 *			Can be utilized to make proportional adjustments to output magnitudes.
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	omega_RADSCOPE
 *	\brief	Radial scoping boundary condition
 *			Utilized to provide ideal standard for potential function as well
 *			as to select points for aperature detection
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	apdet_APWIDTH
 *	\brief	Width of aperature; comparison standard
 *			Utilized to classify aperatures
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	apdet_TRUSTINGNESS
 *	\brief	Coefficeint of trust granted to aperature detection decision making
 *			Large trustingness will allow the aperature detection module to effect
 *			output omega values more
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	pi
 *	\brief	High-accuracy representation of pi constant
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	fndef_FIELDBIAS
 *	\brief	Positive-negative bias factor for potential field calculation
 *			When fndef_FIELDBIAS > 0.5, potentials will be predominantly positive; 
 *			When fndef_FIELDBIAS < 0.5, potentials will be predominantly negative; 
 *			Positive values correspond to directional acceptance; 
 *			Negative values correspond to directional rejection; 
 *			In turn, robot will move away from objects directly behind it and head 
 *			towards the most open part of its current field of view.
 *	\see	_POTENTIAL_FIELD(tht,rad)
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	fndef_LOGITCUTOFF
 *	\brief	Cutoff value for _LOGIT(n) based classifcation
 *	\see	_LOGIT(n)
 */
/*-FUNCTIONAL---------------------------------------------------------------------------*/
/*!	\def 	_LOGIT(n)
 *	\brief	Log-odds function
 *	\param	[in]	n	value between {0<n<1}
 *	\return	value between {-inf<output<inf}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_POTENTIAL_rfang(tht)
 *	\brief	Potential function component from sweep angle, theta
 *			#For angles ||tht|| > pi/2, potentials can be negative
 *	\param	[in]	tht		theta of angular sweep {-pi<tht<pi}
 *	\return	value between {-1<output<1}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_POTENTIAL_rfrad(rad)
 *	\brief	Potential function component from range finder radius value
 *	\param	[in]	rad		value between {0<n<inf}
 *	\return	value between {0<output<1}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_POTENTIAL_FIELD(tht,rad)
 *	\brief	Full potential field function
 *	\param	[in]	tht		theta of angular sweep {-pi<tht<pi}
 *	\param	[in]	rad		value between {0<n<inf}
 *	\return	value between {-2<output<2}
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_REL2HEADING(itht)
 *	\brief	Converts {0<tht<2*pi} range finder thete to {-pi<tht<pi} measurement
 *	\param	[in]	itht	input range finder theta
 *	\return	corrected theta relative to robot heading
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_FRMIDX2THETA(i)
 *	\brief	Converts frame index to appropriate theta
 *	\param	[in]	i		index
 *	\return	frame angle
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_GETOMEGA
 *	\brief	Gets most recent output for omega
 *	\return	omega solution
 */
/*--------------------------------------------------------------------------------------*/
/*!	\def 	_GETVELOC
 *	\brief	Gets most recent output for velocity
 *	\return	veloc solution
 */
/*--------------------------------------------------------------------------------------*/

#define		frames_NEXPECTED			359

#define	 	veloc_LINEARCOEFF			10E0
#define	 	veloc_ANGLESCOPE			pi/8

#define	 	omega_LINEARCOEFF			(double)(1E0)
#define	 	omega_RADSCOPE				300

#define		apdet_APWIDTH				40
#define		apdet_TRUSTINGNESS			0.4

#define 	pi		 					3.1415926535897932384626433832795
#define	 	fndef_FIELDBIAS				4.0
#define	 	fndef_LOGITCUTOFF			1E2

#define		_LOGIT(n)					log(n)-log(1-n)
#define		_POTENTIAL_rfang(tht)		cos(tht)					
#define		_POTENTIAL_rfrad(rad)		(fndef_FIELDBIAS*(rad-omega_RADSCOPE/2)/(rad+omega_RADSCOPE))
										
#define		_POTENTIAL_FIELD(tht,rad)	pow((_POTENTIAL_rfang(tht)*_POTENTIAL_rfrad(rad)),3)
#define		_REL2HEADING(itht)			(itht > pi)?(itht-2*pi):(itht)
#define		_FRMIDX2THETA(i)			(double)((double)(i)/(frames_NEXPECTED)*(2*pi))

#define		_GETOMEGA 					_omegasolution
#define		_GETVELOC					_velocsolution

#endif
