#include "Mapper.hpp"
#include "Utilities.hpp"
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
	logDebugMsg("adding a laser scan to the map", 1);
	this->addToMap(*lidar_scan);
}

void Mapper::handleServo(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const servo_t * servo)
{
	logDebugMsg("setting last servo angle to: " + to_string(servo->angle + M_PI/2.0), 1);
	last_servo_angle = servo->angle + M_PI/2.0;
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
	
	double servo_ang_factor = cos(last_servo_angle);
	//for each laser, calculate the beams to step along
	for(int i = 0; i < lidar_scan.nranges; ++i)
	{
		double scan_dist = lidar_scan.ranges[i];
		if(scan_dist < 0)
			continue;//scan_dist = 10;//assume that the laser has gone at least 15m without hitting anything
		if(scan_dist < 0.5)
			continue;
		scan_dist = scan_dist * servo_ang_factor;

		const SLAM::Pose & start_position = closest_pose;
		//start_position.x = (last_pose.x + velx * i);
		//start_position.y = (last_pose.y + vely * i);
		//start_position.theta = (last_pose.theta + veltheta * i);

		double angle = start_position.theta + lidar_scan.rad0 + lidar_scan.radstep * i;
		double cos_ang = cos(angle);
		double sin_ang = sin(-angle);

		logDebugMsg("range: " + to_string(lidar_scan.ranges[i]), 3);
		logDebugMsg("Range being used: " + to_string(scan_dist), 3);

		double endx = start_position.x + scan_dist * cos_ang;
		double endy = start_position.y + scan_dist * sin_ang;

		double incx = laser_step_size * cos_ang;
		double incy = laser_step_size * sin_ang;

		double cx = start_position.x;
		double cy = start_position.y;

		size_t last_cell = map.convertToGridCoords(cx, cy);
		size_t end_cell = map.convertToGridCoords(endx, endy);
		//ignore the first cell as thats where you are
		for(int j = 0; j < scan_dist/laser_step_size; ++j)
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
		if(0 < lidar_scan.ranges[i])
		{
			logDebugMsg("setting (" + to_string(endx) + " , " + to_string(endy) + ") as full", 2);
			addAsFull(endx, endy);
		}
		else
		{
			logDebugMsg("setting (" + to_string(endx) + " , " + to_string(endy) + ") as empty", 2);
			addAsEmpty(endx, endy);
		}
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
