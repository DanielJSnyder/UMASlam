#ifndef __SLAM_UTILITES_HPP__
#define __SLAM_UTILITES_HPP__

#include <string>
#include <iostream>
#include <cmath>
#include <utility>
#include "Pose.hpp"

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

void rotateIntoGlobalCoordsInPlace(double & x, double & y, double & z, const SLAM::Pose & global_pose)
{
	double temp = x;
	x = x * std::cos(global_pose.theta) - y * std::sin(global_pose.theta);
	y = temp * std::sin(global_pose.theta) + y * std::cos(global_pose.theta);
}

}
#endif
