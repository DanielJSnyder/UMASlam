#ifndef __UMA_SLAM_HPP__
#define __UMA_SLAM_HPP__

#include "Localizer.hpp"
#include "Mapper.hpp"
#include "FakeCompass.hpp"
#include "../lcmtypes/slam_pc_t.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/compass_t.hpp"
#include "../lcmtypes/state_t.hpp"

#include <string>
#include <mutex>
#include <iostream>
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

	void handlePointCloud(const lcm::ReceiveBuffer * rbuf, 
						  const std::string & chan,
						  const SLAM::LCM::slam_pc_t * pc);

	void handleState(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LCM::types::state_t * state);

	void handleGPSData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::gps_t * gps_data);

	void handleFOGData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::fog_t * fog_data);

	void handleCompassData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::compass_t * compass_data);

	void handleIMUData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::imu_t * imu_data);

	const GridMap& getMap();

  void printMap(std::ostream &os);

  void stop(); 

	void run();


private:
	bool end_flag; //used for signaling the end for profiling
	bool reinitialized_fog;
	FakeCompass fake_compass;
  // Angle of north relative to current position in radians
  double compass_north;
  // Same thing as above, but from IMU
  double imu_north;
};
#endif
