#ifndef __POINT_CLOUD_HPP__
#define __POINT_CLOUD_HPP__

#include "../lcmtypes/servo_t.hpp"
#include "../lcmtypes/laser_t.hpp"
#include <lcm/lcm-cpp.hpp>

class PointCloudMaker
{
	std::vector<laser_t> scans;
public:
	void handleLaserScan(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::laser_t * scan);

	void handleServo(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::servo_t * servo);


	void generatePointCloud();
	
};

#endif
