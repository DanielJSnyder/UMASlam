#include "SLAM.hpp"
#include "Constants.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace common::LCM::types;
using namespace SLAM::LCM;

Slam::Slam() : mapper(MIN_X, MAX_X, MIN_Y, MAX_Y, SQUARE_SIZE),
			   localizer(NUM_PARTICLES, PERCENT_PREDICTION_PARTICLES),
			   num_mapped_scans(0),
			   end_flag(false),
			   reinitialized_fog(false),
         compass_north(COMPASS_DEFAULT),
         imu_north(IMU_COMPASS_DEFAULT)
{
	llcm.subscribe(SLAM_POINT_CLOUD_CHANNEL, &Slam::handlePointCloud, this);
	llcm.subscribe(GPS_CHANNEL, &Slam::handleGPSData, this);
	llcm.subscribe(FOG_CHANNEL, &Slam::handleFOGData, this);
  llcm.subscribe(COMPASS_CHANNEL, &Slam::handleCompassData, this);
  llcm.subscribe(IMU_CHANNEL, &Slam::handleIMUData, this);
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
  //cout << "HANDLE FOG" << endl;
	localizer.handleFOGData(rbuf, chan, fog_data);
	if(!reinitialized_fog)
	{
		fake_compass.addFOG(*fog_data);
	}
}

void Slam::handleCompassData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const compass_t * compass_data)
{
  //cout << "HANDLE COMPASS" << endl;
  compass_north = compass_data->yaw;
}

void Slam::handleIMUData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const imu_t * imu_data)
{
  //cout << "HANDLE IMU" << endl;
  //cout << "SETTING IMU NORTH TO " << imu_data->yaw << endl;
  imu_north = imu_data->yaw;
  localizer.handleIMUData(rbuf, chan, imu_data);
}

void Slam::handleGPSData(const lcm::ReceiveBuffer * rbuf,
						 const string & chan,
						 const gps_t * gps_data)
{
  //cout << "HANDLE GPS" << endl;
	localizer.handleGPSData(rbuf, chan, gps_data);

	//reinitialization of the fog
	if(!reinitialized_fog)
	{
    string priority = COMPASS_PRIORITY;
    if(priority == "Compass") 
    {
      //cout << "CHOSE COMPASS" << endl;
      // Use IMU if data is bad, otherwise use compass 
      if(compass_north != COMPASS_DEFAULT) 
      {
        cout << "SETTING HEADING TO " << compass_north << " FROM COMPASS" << endl;
        localizer.reinitializeFOG(compass_north);
        reinitialized_fog = true;
        return;
      }
      else if(imu_north != IMU_COMPASS_DEFAULT)
      {
        cout << "SETTING HEADING TO " << imu_north << " FROM IMU" << endl;
        localizer.reinitializeFOG(imu_north);
        reinitialized_fog = true;
      }

    }
    else if(priority == "IMU") 
    {
      //cout << "CHOSE IMU" << endl;
      // Use compass if data is bad, otherwise use IMU
      if(imu_north != IMU_COMPASS_DEFAULT) 
      {
        cout << "SETTING HEADING TO " << imu_north << " FROM IMU" << endl;
        localizer.reinitializeFOG(imu_north);
        reinitialized_fog = true;
        return;
      }
      else if(compass_north != COMPASS_DEFAULT) {
        cout << "SETTING HEADING TO " << compass_north << " FROM COMPASS" << endl;
        localizer.reinitializeFOG(compass_north);
        reinitialized_fog = true;
      }
    }
    // Fall back to fake compass if the priority is anything other than compass or IMU
    else  
    {
      fake_compass.addGPS(*gps_data);
      if(fake_compass.getDistFromOrigin() > ORIGIN_DIST_BEFORE_REINITIALIZATION)
      {
        //cout << "CHOSE FAKE COMPASS" << endl;
        reinitialized_fog = true;
        mapper.reset();
        localizer.reset();
        localizer.reinitializeFOG(fake_compass.getNorthLocation(localizer.getFogInitialization()));
        cout << "SETTING HEADING TO " << fake_compass.getNorthLocation(localizer.getFogInitialization()) << " FROM FAKE COMPASS" << endl;
        localizer.updateMap(mapper.getMap());
        //cout << "DONE SETTING FAKE COMPASS" << endl;
      }
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

void Slam::printMap(std::ostream &os) 
{
  mapper.printMap(os);
}

void Slam::stop() 
{
  //cout << "END FLAG" << endl;
  end_flag = true;
  //cout << "MAP PRINT" << endl;
  printMap(cout);
}

void Slam::run()
{
  //cout << "BEFORE SLAM LOOP" << endl;
	while(!end_flag)
	{
    //cout << "IN SLAM LOOP" << endl;
		llcm.handle();
	}
  //cout << "PAST SLAM LOOP" << endl;
}
