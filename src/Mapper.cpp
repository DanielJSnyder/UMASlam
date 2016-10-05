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

void Mapper::handlePointCloud(const lcm::ReceivBuffer * rbuf,
							  const string & chan,
							  const slam_pc_t * pc)
{
	SLAM::logDebugMsg("adding a slam point cloud to the map", 1);
	addToMap(*pc);
}

void Mapper::handleState(const lcm::ReceiveBuffer * rbuf,
						 const string & chan, 
						 const state_t * state)
{
	this->addPose(SLAM::Pose(state->x, state->y, state->yaw, state->utime));
}

SLAM::Pose Mapper::findAssociatedPose(int64_t time)
{
	//find the pose closest to the time (assume movement between poses is neglegible)
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
	return closest_pose;
}

void Mapper::addToMap(const slam_pc_t & pc)
{
	for(int scan_num = 0; scan_num < pc.num_scans; ++scan_num)
	{
		SLAM::Pose closest_pose = findAssociatedPose(pc.cloud[scan_num].utime);
		for(int point_num = 0; point_num < pc.cloud[scan_num].scan_size; ++point_num)
		{
			addPointToMap(closest_pose, pc.cloud[scan_num].scan_line[point_num]);
		}
	}
}

void Mapper::addPointToMap(const SLAM::Pose & start_pose, const point3D_t & local_coords_end_point)
{
	//get the global coordinates
	double x = local_coords_end_point.x;
	double y = local_coords_end_point.y;
	double z = local_coords_end_point.z;

	SLAM::rotateIntoGlobalCoordsInPlace(x,y,z,start_pose);

	//only handling 2d case
	double dx = x - start_pose.x;
	double dy = y - start_pose.y;
	double total_dist = std::sqrt(dx * dx + dy * dy);

	int max_num_steps = std::ceil(total_dist/laser_step_size);
	
	size_t end_cell = map.convertToGridCoords(x, y);

	size_t prev_cell = map.convertToGridCoords(start_pose.x, start_pose.y);

	for(int i = 0; i < max_num_step && prev_cell != end_cell; ++i)
	{
		//calculate coords based on similar triangles
		double dist_ratio = i * laser_step_size/total_dist;
		double curr_x = start_position.x + (dx * dist_ratio);
		double curr_y = start_position.y + (dy * dist_ratio);

		size_t cell_num = map.convertToGridCoords(curr_x, curr_y);
		if(cell_num != prev_cell)
		{
			if(cell_num == end_cell)
			{
				addAsFull(curr_x, curr_y);
				prev_cell = end_cell;
			}
			else
			{
				addAsEmpty(curr_x, curr_y);
				prev_cell = cell_num;
			}
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
