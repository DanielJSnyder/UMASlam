#ifndef __UMA_SLAM_POSE_HPP__
#define __UMA_SLAM_POSE_HPP__

namespace SLAM
{

struct Pose
{
	double x;
	double y;
	double theta;
	int64_t utime;

	Pose() : 
		x(0), y(0), theta(0), utime(0) {}

	Pose(double tx, double ty, double tt, int64_t time) : 
		x(tx), y(ty), theta(tt), utime(time) {}
};

}

#endif
