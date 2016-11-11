#include "SLAM.hpp"
#include "Constants.hpp"
#include <iostream>

using namespace std;
using namespace common::LCM::types;
using namespace SLAM::LCM;

Slam::Slam() : mapper(MIN_X, MAX_X, MIN_Y, MAX_Y, SQUARE_SIZE),
			   localizer(NUM_PARTICLES),
			   num_mapped_scans(0),
			   end_flag(false),
			   reinitialized_fog(false)
{
	llcm.subscribe(SLAM_POINT_CLOUD_CHANNEL, &Slam::handlePointCloud, this);
	llcm.subscribe(GPS_CHANNEL, &Slam::handleGPSData, this);
	llcm.subscribe(FOG_CHANNEL, &Slam::handleFOGData, this);
}

void Slam::handlePointCloud(const lcm::ReceiveBuffer * rbuf,
						    const string & chan,
						    const slam_pc_t * pc)
{
	if(num_mapped_scans >= NUM_ONLY_MAP_SCANS)
	{
		localizer.handlePointCloud(rbuf, chan, pc);
		mapper.addPose(localizer.getPose());
	}
	else
	{
		++num_mapped_scans;
	}
	//sets scope for lock guard
	{
		std::lock_guard<std::mutex> map_lock(map_mut);
		mapper.handlePointCloud(rbuf, chan, pc);
		localizer.updateMap(mapper.getMap());
	}

	#ifdef PROFILE
	if(num_mapped_scans >= NUM_ONLY_MAP_SCANS)
	{
		++num_mapped_scans;
	}
	
	if(num_mapped_scans > NUM_PROFILED_SCANS)
	{
		end_flag = true;
	}

	#endif
}

void Slam::handleFOGData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const fog_t * fog_data)
{
	localizer.handleFOGData(rbuf, chan, fog_data);
	if(!reinitialized_fog)
	{
		fake_compass.addFOG(*fog_data);
	}
}

void Slam::handleGPSData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const gps_t * gps_data)
{
	localizer.handleGPSData(rbuf, chan, gps_data);

	//reinitialization of the fog
	if(USE_FAKE_COMPASS && !reinitialized_fog)
	{
		fake_compass.addGPS(*gps_data);
		if(fake_compass.getDistFromOrigin() > ORIGIN_DIST_BEFORE_REINITIALIZATION)
		{
			reinitialized_fog = true;
			mapper.reset();
			localizer.reset();
			localizer.reinitializeFOG(fake_compass.getNorthLocation(localizer.getFogInitialization()));
			localizer.updateMap(mapper.getMap());
		}
	}
}

void Slam::handleState(const lcm::ReceiveBuffer * rbuf,
					   const string & chan,
					   const state_t * state)
{
	//not implemented, may not be needed
}

const GridMap& Slam::getMap() 
{
	std::lock_guard<std::mutex> map_lock(map_mut);
	return mapper.getMap();
}

void Slam::run()
{
	while(!end_flag)
	{
		llcm.handle();
	}
}
