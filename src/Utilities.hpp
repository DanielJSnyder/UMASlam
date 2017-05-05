#ifndef __SLAM_UTILITES_HPP__
#define __SLAM_UTILITES_HPP__

#include <iostream>
#include <cmath>
#include "Pose.hpp"

#define DEG_TO_RAD(d) d/180.0 *M_PI

namespace SLAM
{

/*
	writes msg to screen if the level is less than or equal to the debug level set
*/
template <typename Printable>
void logDebugMsg(const Printable & msg, int level)
{
	#ifdef SLAM_DEBUG_LEVEL
	if(level <= SLAM_DEBUG_LEVEL)
	{
		std::cerr << msg << std::endl;
	}
	#endif
}

static void rotateIntoGlobalCoordsInPlace(double & x, double & y, double & z, const SLAM::Pose & global_pose)
{
	double temp = x;
	double cos_ang = std::cos(global_pose.theta);
	double sin_ang = std::sin(global_pose.theta);
	x = (x * cos_ang - y * sin_ang) + global_pose.x;
	y = (temp * sin_ang + y * cos_ang) + global_pose.y;
}

//        |
//        |
//   2    |   1
//        |
//------------------ 0
//        |
//   3    |   4
//        |
//        |
// angle goes counterclockwise from 0, in radians
//
// atan2 only outputs angles from -pi to pi by default, meaning that the
// angle returned will always be in the 1st or 4th quadrant. This atan function
// corrects for this based on the values of the input variables. The SLAM coordinate
// system redefines y so that the y axis is horizontal, to avoid confusion and to keep
// this function generic, I've avoided using these variable names in this function
//
// arctan(numerator / denominator) = angle in radians
static double quadrantCorrectedAtan(double & numerator, double & denominator) {
  double angle = atan2(numerator, denominator); 
  if(denominator < 0) {
    // In the 2nd or 3rd quadrant, the atan2 angle will be in the 4th or 1st quadrant respectively,
    // so add pi to shift the angle to the correct quadrant
    logDebugMsg("Adjusted angle of point at (" + std::to_string(numerator) + ", " + std::to_string(denominator) + ")", 1);
    angle += M_PI;
  }
  
  // Shift angle into the bounds of the coordinate system [0, 2pi]
	while(angle < 0)
		angle += 2 * M_PI;
	while(angle > 2 * M_PI)
		angle -= 2 * M_PI;

  return angle;
}
}
#endif
