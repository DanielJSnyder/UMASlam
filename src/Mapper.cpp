#include "Mapper.hpp"
#include <iostream>
#include <cmath>

using namespace std;
using namespace common::LCM::types;

Mapper::Mapper(double mnx, double mxx, double mny, double mxy, double ss) :
	map(mnx, mxx, mny, mxy, ss),
	laser_step_size(ss/3.0)
{
	poses.push_back(SLAM::Pose());
}

void Mapper::handleLaserScan(const lcm::ReceiveBuffer * rbuf,
							 const string & chan,
							 const laser_t * lidar_scan)
{
	#ifdef SLAM_DEBUG
	cout << "adding a laser scan to the map" << endl;
	#endif
	this->addToMap(*lidar_scan);
}

void Mapper::handleState(const lcm::ReceiveBuffer * rbuf,
						 const string & chan, 
						 const state_t * state)
{
	this->addPose(SLAM::Pose(state->x, state->y, state->yaw, state->utime));
}

void Mapper::addToMap(const common::LCM::types::laser_t & lidar_scan)
{

	//find the pose closest to the lidar scan (assume no movement during a single laser scan due to large square size
	SLAM::Pose closest_pose = poses.front();
	int64_t t_diff = abs(closest_pose.utime - lidar_scan.utime);
	for(const SLAM::Pose p : poses)
	{
		int64_t diff = abs(p.utime - lidar_scan.utime);
		if(diff < t_diff)
		{
			closest_pose = p;
			t_diff = diff;
		}
	}

	//calculate the velocity between the last two poses
//	double velx = (curr_pose.x - last_pose.x)/ double(lidar_scan.nranges);
//	double vely = (curr_pose.y - last_pose.y)/ double(lidar_scan.nranges);
//	double veltheta = (curr_pose.theta - last_pose.theta)/ double(lidar_scan.nranges);
	
	//for each laser, calculate the beams to step along
	for(int i = 0; i < lidar_scan.nranges; ++i)
	{
		if(lidar_scan.ranges[i] < 1)
			continue;

		const SLAM::Pose & start_position = closest_pose;
		//start_position.x = (last_pose.x + velx * i);
		//start_position.y = (last_pose.y + vely * i);
		//start_position.theta = (last_pose.theta + veltheta * i);

		double angle = start_position.theta + lidar_scan.rad0 + lidar_scan.radstep * i;
		double cos_ang = cos(angle);
		double sin_ang = sin(-angle);

		#ifdef SLAM_DEBUG
		cout << "range: " << lidar_scan.ranges[i] << endl;
		#endif

		double endx = start_position.x + lidar_scan.ranges[i] * cos_ang;
		double endy = start_position.y + lidar_scan.ranges[i] * sin_ang;

		double incx = laser_step_size * cos_ang;
		double incy = laser_step_size * sin_ang;

		double cx = start_position.x;
		double cy = start_position.y;

		size_t last_cell = map.convertToGridCoords(cx, cy);
		size_t end_cell = map.convertToGridCoords(endx, endy);
		//ignore the first cell as thats where you are
		for(int j = 0; j < lidar_scan.ranges[i]/laser_step_size; ++j)
		{
			cx += incx;
			cy += incy;
			size_t cell = map.convertToGridCoords(cx, cy);

			if(last_cell != cell && cell != end_cell)
			{
				if(cell != end_cell)
				{
					addAsEmpty(cx, cy);
				}
				last_cell = cell;
			}
			else if(end_cell == cell)
			{
				break;
			}
		}
		#ifdef SLAM_DEBUG
		cout << "setting (" << endx << " , " << endy << ") as full" << endl;
		#endif
		addAsFull(endx, endy);
	}
}

void Mapper::addPose(const SLAM::Pose & pose)
{
	poses.push_back(pose);
}

void Mapper::addAsEmpty(double x, double y)
{
	if(map.at(x,y) < abs(EMPTY_INC))
	{
		map.at(x,y) = 0;
	}
	else
	{
		map.at(x,y) += EMPTY_INC;
	}
}

void Mapper::addAsFull(double x, double y)
{
	if(255 < ((int16_t)(map.at(x,y)) + FULL_INC))
	{
		map.at(x,y) = 255;
	}
	else
	{
		map.at(x,y) += FULL_INC;
	}
}

GridMap Mapper::getMapCopy() const
{
	return map;
}
