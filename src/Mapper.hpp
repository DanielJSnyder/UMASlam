#ifndef __UMA_SLAM_MAPPER_HPP__
#define __UMA_SLAM_MAPPER_HPP__

#include "GridMap.hpp"
#include "Pose.hpp"
#include <lcm/lcm-cpp.hpp>
#include "../lcmtypes/laser_t.hpp"

class Mapper
{
public:
	Mapper(double mnx, double mxx, double mny, double mxy, double ss);

	void handleLaserScan(const lcm::ReceiveBuffer * rbuf, 
						 const std::string & chan,
						 const common::LCM::types::laser_t & lidar_scan);

	void addToMap(const common::LCM::types::laser_t & lidar_scan);

	GridMap getMapCopy() const;

	void addPose(const SLAM::Pose & pose);

private:
	void addAsEmpty(double x, double y);
	void addAsFull(double x, double y);
	
	std::vector<SLAM::Pose> poses;
	GridMap map;
	double laser_step_size;

	const int8_t FULL_INC = 1;
	const int8_t EMPTY_INC = -1;
};

#endif
