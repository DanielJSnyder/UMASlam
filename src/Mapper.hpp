#ifndef __UMA_SLAM_MAPPER_HPP__
#define __UMA_SLAM_MAPPER_HPP__

#include "GridMap.hpp"
#include "Pose.hpp"
#include "Constants.hpp"
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <iostream>
#include "../lcmtypes/laser_t.hpp"
#include "../lcmtypes/state_t.hpp"
#include "../lcmtypes/slam_pc_t.hpp"

struct GridUpdate
{
	size_t grid_index;
	double value;
};

class Mapper
{
public:
	Mapper(double mnx, double mxx, double mny, double mxy, double ss);

	void handlePointCloud(const lcm::ReceiveBuffer * rbuf,
						  const std::string & chan,
						  const SLAM::LCM::slam_pc_t * pc);

	void handleState(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::state_t * state);

	void addToMap(const SLAM::LCM::slam_pc_t & pc);

	void addPointToMap(const SLAM::Pose & curr_pose, const SLAM::LCM::point3D_t & local_coords_end_point, int8_t hit);

	GridMap getMapCopy() const;

	const GridMap& getMap() const;

	void addPose(const SLAM::Pose & pose);
	void reset();

  void printMap(std::ostream &os);

private:
	SLAM::Pose findAssociatedPose(int64_t time);
	void addAsEmpty(double x, double y);
	void addAsFull(double x, double y);
	int findUpdate(std::size_t idx);
	void updateMap();
  int64_t utime_now();
  void publishMap();
	
	std::vector<SLAM::Pose> poses;
	std::vector<GridUpdate> grid_updates;
	GridMap map;
	double laser_step_size;

	const int8_t FULL_INC = FULL_SQUARE_INC;
	const int8_t EMPTY_INC = EMPTY_SQUARE_INC;
};

#endif
