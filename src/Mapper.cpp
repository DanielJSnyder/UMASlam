#include "Mapper.hpp"
#include "Utilities.hpp"
#include <iostream>
#include <cmath>
#include <sys/time.h>

using namespace std;
using namespace common::LCM::types;
using namespace SLAM::LCM;

Mapper::Mapper(double mnx, double mxx, double mny, double mxy, double ss) :
	map(mnx, mxx, mny, mxy, ss),
	laser_step_size(ss/3.0)
{
	poses.push_back(SLAM::Pose());
}

void Mapper::handlePointCloud(const lcm::ReceiveBuffer * rbuf,
							  const string & chan,
							  const slam_pc_t * pc)
{
  //cout << "HANDLE POINT CLOUD IN MAPPER" << endl;
	SLAM::logDebugMsg("adding a slam point cloud to the map", 1);
	addToMap(*pc);
}

void Mapper::handleState(const lcm::ReceiveBuffer * rbuf,
						 const string & chan, 
						 const state_t * state)
{
  //cout << "HANDLE STATE IN MAPPER" << endl;
	this->addPose(SLAM::Pose(state->x, state->y, state->yaw, state->utime));
}

SLAM::Pose Mapper::findAssociatedPose(int64_t time)
{
	//find the pose closest to the time (assume movement between poses is neglegible)
	size_t i = 0;
	for(i = 0; i < poses.size(); ++i)
	{
		if(time < poses[i].utime)
		{
			break;
		}
	}

	//i now represents the pose after the lidar scan time
	SLAM::Pose p1;
	SLAM::Pose p2;
	if(i == 0)
	{
		return poses.front();
	}
	else if(poses.size() == i)
	{
		p1 = poses[i-2];
		p2 = poses[i-1];
	}
	else
	{
		p1 = poses[i-1];
		p2 = poses[i];
	}

	int64_t diff_utime = p2.utime - p1.utime;
	double diff_x = p2.x - p1.x;
	double diff_y = p2.y - p1.y;
	double diff_theta = p2.theta - p1.theta;
	if(diff_theta > M_PI)
	{
		diff_theta -= 2.0*M_PI;
	}
	else if(diff_theta < -M_PI)
	{
		diff_theta += 2.0*M_PI;
	}
	double proportion = 
		static_cast<double>(time - p1.utime)/(static_cast<double>(diff_utime));
	return SLAM::Pose(proportion*(diff_x) + p1.x,
					  proportion*(diff_y) + p1.y,
					  proportion*(diff_theta) + p1.theta,
					  time);
}
void Mapper::addToMap(const slam_pc_t & pc)
{
	grid_updates.clear();
  //cout << "After grid update in Mapper" << endl;
	for(int scan_num = 0; scan_num < pc.num_scans; ++scan_num)
	{
		SLAM::Pose closest_pose = findAssociatedPose(pc.cloud[scan_num].utime);
		for(int point_num = 0; point_num < pc.cloud[scan_num].scan_size; ++point_num)
		{
			addPointToMap(closest_pose, pc.cloud[scan_num].scan_line[point_num], pc.cloud[scan_num].hit[point_num]);
		}
	}
  //cout << "After finding associated poses in Mapper" << endl;
	updateMap();
  //cout << "After updating map in Mapper" << endl;
}

void Mapper::addPointToMap(const SLAM::Pose & start_pose, const point3D_t & local_coords_end_point, int8_t hit)
{
	//get the global coordinates
	double x = local_coords_end_point.x;
	double y = local_coords_end_point.y;
	double z = local_coords_end_point.z;
	double angle = atan2(y,x);
	if(abs(angle * 180.0/M_PI ) > LIDAR_MAP_RANGE_DEG)
	{
//		if(hit == 1)
//		{
//			SLAM::rotateIntoGlobalCoordsInPlace(x,y,z,start_pose);
//			addAsFull(x,y);
//		}
		return;
	}

  SLAM::rotateIntoGlobalCoordsInPlace(x,y,z,start_pose);

  //only handling 2d case
  double dx = x - start_pose.x;
  double dy = y - start_pose.y;
  double total_dist = std::sqrt(dx * dx + dy * dy);

  int max_num_steps = std::ceil(total_dist/laser_step_size);
  
  size_t end_cell = map.convertToGridCoords(x, y);

  size_t prev_cell = map.convertToGridCoords(start_pose.x, start_pose.y);

  for(int i = 0; i < max_num_steps && prev_cell != end_cell; ++i)
  {
    //calculate coords based on similar triangles
    double dist_ratio = i * laser_step_size/total_dist;
    double curr_x = start_pose.x + (dx * dist_ratio);
    double curr_y = start_pose.y + (dy * dist_ratio);

    size_t cell_num = map.convertToGridCoords(curr_x, curr_y);
    if(cell_num != prev_cell)
    {
      if(cell_num != end_cell)
      {
        if(curr_x < 0) {
          //cout << "NEGATIVE CURR_X" << endl;
        }
        else if(curr_y < 0) {
          //cout << "NEGATIVE CURR_Y" << endl;
        }
        addAsEmpty(curr_x, curr_y);
      }

      prev_cell = cell_num;
    }
  }
  if(hit == 1)
  {
    addAsFull(x, y);
  }
  else
  {
    addAsEmpty(x,y);
  }
}

void Mapper::addPose(const SLAM::Pose & pose)
{
	poses.push_back(pose);
}

void Mapper::addAsEmpty(double x, double y)
{
	size_t grid_idx = map.convertToGridCoords(x,y);
	int update_index = findUpdate(grid_idx);
	if(update_index == -1)
	{
		GridUpdate u;
    if(grid_updates.empty()) {
      //cout << "FIRST GRID SQUARE INDEX EMPTY: " << grid_idx << endl;
      //cout << "FIRST GRID SQUARE INDEX EMPTY X: " << x << endl;
      //cout << "FIRST GRID SQUARE INDEX EMPTY Y: " << y << endl;
    }
		u.grid_index = grid_idx;
		u.value = EMPTY_INC;

		grid_updates.push_back(u);
	}
	else
	{
		if(grid_updates[update_index].value <= 0)
		{
			grid_updates[update_index].value += EMPTY_SQUARE_INC;
		}
	}
}

void Mapper::addAsFull(double x, double y)
{
	size_t grid_idx = map.convertToGridCoords(x,y);
	int update_index = findUpdate(grid_idx);
	if(update_index == -1)
	{
		GridUpdate u;
    if(grid_updates.empty()) {
      //cout << "FIRST GRID SQUARE INDEX FULL: " << grid_idx << endl;
      //cout << "FIRST GRID SQUARE INDEX FULL X: " << x << endl;
      //cout << "FIRST GRID SQUARE INDEX FULL Y: " << y << endl;
    }
		u.grid_index = grid_idx;
		u.value = FULL_INC;
		grid_updates.push_back(u);
	}
	else
	{
		if(grid_updates[update_index].value < 0)
		{
			grid_updates[update_index].value = FULL_INC;
		}
		else
			grid_updates[update_index].value += FULL_INC;
	}
}

int Mapper::findUpdate(size_t idx)
{
	for(size_t i = 0; i < grid_updates.size(); ++i)
	{
		if(grid_updates[i].grid_index == idx)
		{
			return i;
		}
	}
	return -1;
}

void Mapper::updateMap()
{
  //cout << "Grid update size = " << grid_updates.size() << endl;
	for(GridUpdate & u : grid_updates)
	{
		//clamp the values here
    //cout << "Start of the GridUpdate loop" << endl;
    //cout << "u.grid_index = " << u.grid_index << endl;
    //cout << "u.value = " << u.value << endl;
		double value = map[u.grid_index];
    //cout << "After value set: ";
    //cout << value << endl;
		value += u.value;
    //cout << "After value add: ";
    //cout << u.value << endl;
		int16_t result = std::min((int64_t)255, std::max((int64_t)0, static_cast<int64_t>(value)));
    //cout << "Result: ";
    //cout << result << endl;
		map[u.grid_index] = result;
    //cout << "Set map to "; 
    //cout << map[u.grid_index] << endl;
	}
}

void Mapper::reset()
{
	map.resetMap();
	poses.clear();
}

int64_t Mapper::utime_now() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void Mapper::publishMap() {
  map.publishMap(utime_now(), SLAM_MAP_CHANNEL);
}

GridMap Mapper::getMapCopy() const
{
	return map;
}

const GridMap& Mapper::getMap() const
{
	return map;
}
