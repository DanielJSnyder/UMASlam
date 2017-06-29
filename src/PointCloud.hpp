#ifndef __POINT_CLOUD_HPP__
#define __POINT_CLOUD_HPP__

#include "../lcmtypes/servo_t.hpp"
#include "../lcmtypes/laser_t.hpp"
#include "../lcmtypes/slam_pc_t.hpp"
#include <lcm/lcm-cpp.hpp>

class PointCloudMaker
{
	std::vector<common::LCM::types::laser_t> scans;
	std::vector<common::LCM::types::servo_t> servos;
	int32_t last_dir;
	SLAM::LCM::slam_pc_t publish_pc;

public:
	PointCloudMaker();

	void handleLaserScan(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::laser_t * scan);

	void handleServo(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::servo_t * servo);


	void extractPointCloud();
	
private:
	SLAM::LCM::point3D_t createPoint(int64_t utime, double range, double servo_angle, double point_angle);
};

#endif
