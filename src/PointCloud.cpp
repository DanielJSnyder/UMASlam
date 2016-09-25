#include "PointCloud.hpp"
#include "Constants.hpp"
#include "Utilities.hpp"
#include "../lcmtypes/point_cloud_t.hpp"
#include <cmath>

using namespace std;
using namespace common::LCM::types;

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
	size_t row_size = 0;
	size_t col_size = 0;
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
			logDebugMsg("ERROR: PRIOR SERVO NOT FOUND", -1);
			continue;
		}

		//assume that the utime is the start of the lidar scan
		//calculate at what angle the first lidar point was received
		int64_t delta_time = l.utime - prior_servo.utime;
		double initial_angle = prior_servo.angle + prior_servo.a_vel * delta_time/10e6;

		//NOT OPTIMAL WAY TO FILL THIS
		std::vector<double> x_vect;
		std::vector<double> y_vect;
		std::vector<double> z_vect;
		for(int i = 0; i < l.nranges; ++i)
		{
			if(l.ranges[i] > 0.5)
			{
				logDebugMsg("LOGGING A RANGE GREATER THAN 0.5m", 1);
				double phi = l.radstep * (double)i + l.rad0;
				double theta = initial_angle;//phi is off of z axis
				double r = l.ranges[i];
				
				double sin_ang = sin(theta);
				double x = -r*sin_ang*cos(phi);
				double y = r*sin_ang*sin(phi);
				double z = r*cos(theta);

				x_vect.push_back(x);
				y_vect.push_back(y);
				z_vect.push_back(z);
			}
			else
			{
				x_vect.push_back(0);
				y_vect.push_back(0);
				z_vect.push_back(0);
			}
		}
		publish_pc.x.push_back(x_vect);
		publish_pc.y.push_back(y_vect);
		publish_pc.z.push_back(z_vect);
		col_size = l.nranges;
		++row_size;

		publish_pc.utime = l.utime;
	}
	publish_pc.row_size = row_size;
	publish_pc.col_size = col_size;

	lcm::LCM lcm;
	lcm.publish(SLAM_POINT_CLOUD_CHANNEL, &publish_pc);

	//remove all scans that were used
	scans.clear();
	servos.clear();
}
