#include "SLAM.hpp"
#include <iostream>

using namespace std;
using namespace common::LCM::types;
Slam::Slam() : mapper(-50, 50, -50, 50, .5),
			   localizer(1000)
{
	llcm.subscribe("SENSOR_LASER", &Slam::handleLaserScan, this);
	llcm.subscribe("SENSOR_GPS", &Slam::handleGPSData, this);
	llcm.subscribe("SENSOR_FOG", &Slam::handleFOGData, this);
	llcm.subscribe("SENSOR_LASER_SERVO", &Slam::handleServo, this);
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
	localizer.handleLaserScan(rbuf, chan, lidar_scan);
	mapper.addPose(localizer.getPose());
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
