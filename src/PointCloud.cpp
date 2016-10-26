#include "PointCloud.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include "../lcmtypes/point_cloud_t.hpp"
#include <cmath>

using namespace std;
using namespace common::LCM::types;
using namespace SLAM::LCM;

PointCloudMaker::PointCloudMaker() :
	scans(0),
	servos(0),
	last_dir(0)
{
}
									

void PointCloudMaker::handleLaserScan(const lcm::ReceiveBuffer * rbuf,
									  const string & chan,
									  const laser_t * scan)
{
	scans.push_back(*scan);	
}

void PointCloudMaker::handleServo(const lcm::ReceiveBuffer * rbuf,
								  const string & chan,
								  const servo_t * servo)
{
	servos.push_back(*servo);
	if(servo->dir != last_dir)
	{
		if(last_dir != 0) //if this is not the first servo received
		{
			extractPointCloud();
		}
		last_dir = servo->dir;
	}
}

void PointCloudMaker::extractPointCloud()
{
	//zero out the old point_cloud
	publish_pc.utime = 0;
	publish_pc.num_scans = 0;
	publish_pc.cloud.clear();

	for(laser_t & l : scans)
	{
		//find the servo prior to it
		servo_t prior_servo;
		prior_servo.dir = 0;
		for(size_t i = 0; i < servos.size(); ++i)
		{
			if(servos[i].utime < l.utime)
			{
				prior_servo = servos[i];
			}
			else
			{
				break;
			}
		}
		if(prior_servo.dir == 0)
		{
			SLAM::logDebugMsg("ERROR: PRIOR SERVO NOT FOUND", -1);
			continue;
		}


		//assume that the utime is the start of the lidar scan
		//calculate at what angle the first lidar point was received
		int64_t delta_time = l.utime - prior_servo.utime;
		double initial_angle = prior_servo.angle + prior_servo.a_vel * delta_time/10e6;

		//create the scan line being built
		scan_line_t curr_scan;
		curr_scan.utime = l.utime;
		curr_scan.scan_size = l.nranges;
		curr_scan.scan_line.resize(curr_scan.scan_size);
		curr_scan.hit.resize(curr_scan.scan_size);

		for(int i = 0; i < l.nranges; ++i)
		{
			point3D_t point;
			point.utime = l.utime;
			double r = l.ranges[i];
			bool hit = true;
			if(r < 0)
			{
				r = DEFAULT_MISS_RANGE;
				hit = false;
			}

			double phi = l.radstep * (double)i + l.rad0;
			double theta = initial_angle;//theta is off of z axis
			
			double sin_ang = sin(theta);
			double x = -r*sin_ang*cos(phi);
			double y = r*sin_ang*sin(phi);
			double z = r*cos(theta);

			//add the point to the point_cloud
			point.x = x;
			point.y = y;
			point.z = z;

			curr_scan.scan_line[i] = point;
			curr_scan.hit[i] = (hit)? 1 : 0;
		}

		//add the scan to the point_cloud
		publish_pc.cloud.push_back(curr_scan);
	}

	publish_pc.num_scans = publish_pc.cloud.size();
	lcm::LCM lcm;
	lcm.publish(SLAM_POINT_CLOUD_CHANNEL, &publish_pc);

	//remove all scans that were used
	scans.clear();
	servos.clear();
}
