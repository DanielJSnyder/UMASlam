#include <string>
#include <iostream>
#include <cmath>
#include <utility>
#include "Utilities.hpp"

namespace SLAM
{
void rotateIntoGlobalCoordsInPlace(double & x, double & y, double & z, const SLAM::Pose & global_pose)
{
	double temp = x;
	x = x * std::cos(global_pose.theta) - y * std::sin(global_pose.theta);
	y = temp * std::sin(global_pose.theta) + y * std::cos(global_pose.theta);
}

}
