#ifndef __UMA_SLAM_HPP__
#define __UMA_SLAM_HPP__

#include "Localizer.hpp"
#include "Mapper.hpp"
#include "../lcmtypes/laser_t.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/servo_t.hpp"
#include "../lcmtypes/state_t.hpp"

#include <string>
#include <mutex>
#include <lcm/lcm-cpp.hpp>

class Slam
{
	Mapper mapper;
	Localizer localizer;
	std::mutex map_mut;
	lcm::LCM llcm;

	size_t num_mapped_scans;

public:
	Slam();

	void handleLaserScan(const lcm::ReceiveBuffer * rbuf, 
						 const std::string & chan,
						 const common::LCM::types::laser_t * lidar_scan);

	void handleServo(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::servo_t * servo);

	void handleState(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::state_t * state);

	void handleGPSData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::gps_t * gps_data);

	void handleFOGData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::fog_t * fog_data);

	GridMap getMap();

	void run();
};
#endif
