#ifndef __UMA_SLAM_MAPPER_HPP__
#define __UMA_SLAM_MAPPER_HPP__

#include "GridMap.hpp"
#include "Pose.hpp"
#include <lcm/lcm-cpp.hpp>
#include <string>
#include "../lcmtypes/laser_t.hpp"
#include "../lcmtypes/state_t.hpp"
#include "../lcmtypes/servo_t.hpp"

class Mapper
{
public:
	Mapper(double mnx, double mxx, double mny, double mxy, double ss);

	void handleLaserScan(const lcm::ReceiveBuffer * rbuf, 
						 const std::string & chan,
						 const common::LCM::types::laser_t * lidar_scan);

	void handleServo(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::servo_t * servo);

	void handleState(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::state_t * state);

	void addToMap(const common::LCM::types::laser_t & lidar_scan);

	GridMap getMapCopy() const;

	void addPose(const SLAM::Pose & pose);

private:
	void addAsEmpty(double x, double y);
	void addAsFull(double x, double y);
	
	std::vector<SLAM::Pose> poses;
	GridMap map;
	double laser_step_size;
	double last_servo_angle;

	const int8_t FULL_INC = 10;
	const int8_t EMPTY_INC = -1;
};

#endif
