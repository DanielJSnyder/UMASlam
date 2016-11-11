#include "Localizer.hpp"
#include "Constants.hpp"
#include "../lcmtypes/state_t.hpp"
#include "Utilities.hpp"
#include "../lcmtypes/particles_t.hpp"
#include <queue>

using namespace std;
using namespace common::LCM::types;
using namespace SLAM::LCM;

Particle::Particle() : x(0), y(0), theta(0), likelihood(0)
{
}

Localizer::Localizer(int num_particles) : 
	Localizer(num_particles ,DEFAULT_GPS_SIGMA,DEFAULT_FOG_SIGMA)
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
		initial_theta = DEG_TO_RAD(fog_data->data);
		fog_initialized = true;
	}

	fillParticles(DEG_TO_RAD(fog_data->data) - initial_theta);
}

void Localizer::handlePointCloud(const lcm::ReceiveBuffer * rbuf,
								 const string & chan,
								 const slam_pc_t * pc)
{
	weightParticles(*pc);
}

void Localizer::weightParticles(const slam_pc_t & pc)
{
	for(Particle & p: particles)
	{
		double curr_particle_likelihood = 0;
		SLAM::Pose particle_pose(p.x, p.y,p.theta, 0);

		//for each scan line and each end point do hit or miss
		SLAM::logDebugMsg("point_cloud size: " + to_string(pc.cloud.size()) + "\n", 1);
		for(size_t i = 0; i < pc.cloud.size(); ++i)
		{
			SLAM::logDebugMsg("scan size: " + to_string(pc.cloud[i].scan_line.size()) + "\n", 1);
			for(size_t j = 0; j < pc.cloud[i].scan_line.size(); ++j)
			{
				if(pc.cloud[i].hit[j] == 0)
					continue;
				double x = pc.cloud[i].scan_line[j].x;
				double y = pc.cloud[i].scan_line[j].y;
				double z = pc.cloud[i].scan_line[j].z;
				if(x < 0)
					continue;
			
				SLAM::rotateIntoGlobalCoordsInPlace(x,y,z, particle_pose);
				
				//just do simple hit or miss
				if(map.at(x, y) > HIT_THRESHOLD)//if the square is considered full, add to likelihood
				{ curr_particle_likelihood += HIT_LIKELIHOOD_INC_VALUE; }
			}
		}
		p.likelihood = curr_particle_likelihood;
	}

	setPose(pc.utime);
	publishPose();
	publishParticles();
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

void Localizer::setPose(int64_t utime)
{
	priority_queue<Particle> part_queue;

	size_t p = 0;

	for(; p < NUM_AVERAGE_PARTICLES; ++p)
	{
		part_queue.push(particles[p]);
	}
	
	for(; p < particles.size(); ++p)
	{
		if(part_queue.top().likelihood < particles[p].likelihood)
		{
			part_queue.pop();
			part_queue.push(particles[p]);
		}
	}

	double total_x = 0; 
	double total_y = 0;
	double total_theta = 0;

	while(!part_queue.empty())
	{
		total_x += part_queue.top().x;
		total_y += part_queue.top().y;
		total_theta += part_queue.top().theta;
		part_queue.pop();
	}

	last_pose.x = total_x/NUM_AVERAGE_PARTICLES;
	last_pose.y = total_y/NUM_AVERAGE_PARTICLES;
	last_pose.theta = total_theta/NUM_AVERAGE_PARTICLES;
	last_pose.utime = utime;
}

void Localizer::publishPose() const
{
	state_t pub_state;
	pub_state.x = last_pose.x;
	pub_state.y = last_pose.y;
	pub_state.yaw = last_pose.theta;
	lcm::LCM l;
	l.publish(SLAM_STATE_CHANNEL, &pub_state);
}

void Localizer::publishParticles() const
{
	particles_t curr_particles;
	curr_particles.utime = last_pose.utime;
	curr_particles.num_particles = particles.size();
	for(size_t i = 0; i < particles.size(); ++i)
	{
		particle_t particle;
		particle.utime = last_pose.utime;
		particle.x = particles[i].x;
		particle.y = particles[i].y;
		particle.theta = particles[i].theta;
		particle.likelihood = particles[i].likelihood;
		curr_particles.particles.push_back(particle);
	}

	lcm::LCM l;
	l.publish(SLAM_PARTICLE_CHANNEL, &curr_particles);
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

void Localizer::reset()
{
	fog_initialized = false;
}

void Localizer::reinitializeFOG(double new_initial_fog)
{
	initial_theta = new_initial_fog;
	fog_initialized = true;
}

double Localizer::getFogInitialization() const
{
	return initial_theta;
}
