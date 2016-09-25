#include "SLAM.hpp"
#include "Constants.hpp"
#include <iostream>

using namespace std;
using namespace common::LCM::types;
Slam::Slam() : mapper(-50, 50, -50, 50, .5),
			   localizer(1000),
			   num_mapped_scans(0)
{
	llcm.subscribe(LASER_SCAN_CHANNEL, &Slam::handleLaserScan, this);
	llcm.subscribe(GPS_CHANNEL, &Slam::handleGPSData, this);
	llcm.subscribe(FOG_CHANNEL, &Slam::handleFOGData, this);
	llcm.subscribe(SERVO_CHANNEL, &Slam::handleServo, this);
}

void Slam::handleLaserScan(const lcm::ReceiveBuffer * rbuf,
						   const string & chan,
						   const laser_t * lidar_scan)
{
	{
		std::lock_guard<std::mutex> map_lock(map_mut);
		mapper.handleLaserScan(rbuf, chan, lidar_scan);
		localizer.updateMap(mapper.getMapCopy());
	}
	if(num_mapped_scans >15)
	{
		localizer.handleLaserScan(rbuf, chan, lidar_scan);
		mapper.addPose(localizer.getPose());
	}
	else
	{
		++num_mapped_scans;
	}
}

void Slam::handleFOGData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const fog_t * fog_data)
{
	localizer.handleFOGData(rbuf, chan, fog_data);
}

void Slam::handleGPSData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const gps_t * gps_data)
{
	localizer.handleGPSData(rbuf, chan, gps_data);
}

void Slam::handleServo(const lcm::ReceiveBuffer * rbuf,
					   const string & chan,
					   const servo_t * servo)
{
	mapper.handleServo(rbuf, chan, servo);
}

void Slam::handleState(const lcm::ReceiveBuffer * rbuf,
					   const string & chan,
					   const state_t * state)
{
	//not implemented, may not be needed
}

GridMap Slam::getMap() 
{
	std::lock_guard<std::mutex> map_lock(map_mut);
	return mapper.getMapCopy();
}

void Slam::run()
{
	while(true)
	{
		cout << "handling" << endl;
		llcm.handle();
	}
}
