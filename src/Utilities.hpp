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
}
#endif
