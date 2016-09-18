#include "Localizer.hpp"
#include "../lcmtypes/state_t.hpp"

using namespace std;
using namespace common::LCM::types;

Particle::Particle() : x(0), y(0), theta(0), likelihood(0)
{
}

Localizer::Localizer(int num_particles) : particles(num_particles),
										  x_gps_dist(0,1.5),
										  y_gps_dist(0, 1.5),
										  theta_fog_dist(0, 0.5),
										  fog_initialized(false)
{
}

Localizer::Localizer(int num_particles, double gps_sigma, double fog_sigma): 
	particles(num_particles),
	x_gps_dist(0, gps_sigma),
	y_gps_dist(0, gps_sigma),
	theta_fog_dist(0, fog_sigma),
	fog_initialized(false)
{
}

SLAM::Pose Localizer::getPose() const
{
	return last_pose;
}

void Localizer::updateMap(const GridMap & new_map)
{
	map = new_map;
}

void Localizer::handleGPSData(const lcm::ReceiveBuffer * rbuf,
							  const string & chan,
							  const gps_t * gps_data)
{
	//if not initialized, initialize the coordinate transformation
	if(!coord_transformer.isInitialized())
	{
		coord_transformer.initialize(gps_data->latitude, gps_data->longitude);
	}
	else
	{
		//setup the particles to be based on a normal distribution
		fillParticles(*gps_data);
	}
}

void Localizer::handleFOGData(const lcm::ReceiveBuffer * rbuf,
							  const string & chan,
							  const fog_t * fog_data)
{
	if(!fog_initialized)
	{
		initial_theta = fog_data->data;
		fog_initialized = true;
	}

	fillParticles((fog_data->data) - initial_theta);
}

void Localizer::handleLaserScan(const lcm::ReceiveBuffer * rbuf,
								const string & chan,
								const laser_t * laser_scan)
{
	weightParticles(*laser_scan);
}

void Localizer::weightParticles(const laser_t & laser_scan)
{
	for(Particle & p: particles)
	{
		Particle curr_particle  = p;
		double curr_particle_likelihood = 0;
		for(int i = 0; i < laser_scan.nranges; ++i)
		{
			//find the endpoint for the laser scan
			double scan_dist = laser_scan.ranges[i];
			if(scan_dist < 0.5)//don't do anything if the laser point is close to the boat in case of an errant scan
				continue;

			double angle = curr_particle.theta + laser_scan.rad0 + laser_scan.radstep * i;
			double cos_ang = cos(angle);
			double sin_ang = sin(-angle);
			
			double endx = curr_particle.x + scan_dist * cos_ang;
			double endy = curr_particle.y + scan_dist * sin_ang;
			
			//just do simple hit or miss
			if(map.at(endx, endy) > 150)//if the square is considered full, add to likelihood
			{
				curr_particle_likelihood += LIKELIHOOD_INCREMENT_VALUE;
			}
		}
		p.likelihood = curr_particle_likelihood;
	}
	boundLikelihoods();
	setPose();
	publishPose();
}

void Localizer::boundLikelihoods()
{
	double total_particle_likelihood = 0;
	for(Particle & p : particles)
	{
		total_particle_likelihood += p.likelihood;
	}
	for(Particle & p : particles)
	{
		p.likelihood /= total_particle_likelihood;
	}
}

void Localizer::setPose()
{
	Particle best_particle = particles[0];
	for(size_t p = 1; p < particles.size(); ++p)
	{
		if(particles[p].likelihood > best_particle.likelihood)
		{
			best_particle = particles[p];
		}
	}

	last_pose.x = best_particle.x;
	last_pose.y = best_particle.y;
	last_pose.theta = best_particle.theta;
	last_pose.utime = 0;
}

void Localizer::publishPose() const
{
	state_t pub_state;
	pub_state.x = last_pose.x;
	pub_state.y = last_pose.y;
	pub_state.yaw = last_pose.theta;
	lcm::LCM l;
	l.publish("SLAM_STATE", &pub_state);
}

void Localizer::fillParticles(const gps_t & gps_data)
{
	pair<double, double> coords = 
		coord_transformer.transform(gps_data.latitude, gps_data.longitude);

	random_device rd;
	mt19937 gen(rd());
	for(size_t i = 0; i < particles.size(); ++i)
	{
		particles[i].x = coords.first + x_gps_dist(gen);
		particles[i].y = coords.second + y_gps_dist(gen);
	}
}

void Localizer::fillParticles(double theta)
{
	random_device rd;
	mt19937 gen(rd());

	for(size_t i = 0; i < particles.size(); ++i)
	{
		particles[i].theta = theta + theta_fog_dist(gen);
	}
}
