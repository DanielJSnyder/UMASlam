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

Localizer::Localizer(int num_particles, double predict_percent) : 
	Localizer(num_particles, predict_percent, DEFAULT_GPS_SIGMA, DEFAULT_FOG_SIGMA)
{
}

Localizer::Localizer(int num_particles, double predict_percent, double gps_sigma, double fog_sigma): 
	particles(num_particles),
	x_gps_dist(0, gps_sigma),
	y_gps_dist(0, gps_sigma),
	theta_fog_dist(0, fog_sigma),
	x_predict_dist(0, X_PREDICTION_SIGMA),
	y_predict_dist(0, Y_PREDICTION_SIGMA),
	num_predict_particles(num_particles *predict_percent),
	last_utime(0),
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

	last_coord = coord_transformer.transform(gps_data->latitude, gps_data->longitude);
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


	last_theta = (DEG_TO_RAD(fog_data->data) - initial_theta);
}

void Localizer::handlePointCloud(const lcm::ReceiveBuffer * rbuf,
								 const string & chan,
								 const slam_pc_t * pc)
{
	weightParticles(*pc);
}

void Localizer::weightParticles(const slam_pc_t & pc)
{
	createParticles(pc.utime);
	previous_gen_coord = last_coord;
	last_utime = pc.utime;
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
				
				//just do simple hit or miss as lidar sigma < 30mm from data sheet
				if(map.at(x, y) > HIT_THRESHOLD)//if the square is considered full, add to likelihood
				{ curr_particle_likelihood += HIT_LIKELIHOOD_INC_VALUE; }
			}
		}
		p.likelihood = curr_particle_likelihood;
	}

	boundLikelihoods();
	setPose(pc.utime);
	publishPose();
	//publishParticles();
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

//	size_t max_likelihood_location = 0;
//	double max_likelihood = particles[0].likelihood;
//	for(size_t i = 1; i < particles.size(); ++i)
//	{
//		if(particles[i].likelihood > max_likelihood)
//		{
//			max_likelihood = particles[i].likelihood;
//			max_likelihood_location = i;
//		}
//	}
//	if (max_likelihood_location < num_predict_particles)
//	{
//		cout << "Chose a predicted particle" << endl;
//	}
//	else
//	{
//		cout << "Did not choose a predicted particle" << endl;
//	}
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

void Localizer::createPredictionParticles(int64_t curr_utime)
{
	//calculate prediction params
	double dt = curr_utime - last_utime;
	double vel_x = (last_coord.first - previous_gen_coord.first)/(dt);
	double vel_y = (last_coord.second - previous_gen_coord.second)/dt;
	random_device rd;
	mt19937 gen(rd());

	std::vector<Particle> temp_particles(num_predict_particles);
	
	//select the particles that will be predicted forward
	//create the vector of likelihoods (assume likelihoods are bounded 
	size_t particle_index = 0;
	size_t num_sampled = 0;
	double total_likelihood = particles[0].likelihood;
	for(double curr_likelihood = 0.0; 
		num_sampled < num_predict_particles && particle_index < particles.size();
		curr_likelihood += 1.0/num_predict_particles)
	{
		while(curr_likelihood > total_likelihood && particle_index < particles.size())
		{
			total_likelihood += particles[particle_index].likelihood;
			++particle_index;
		}
		if(particle_index < particles.size())
		{
			temp_particles[num_sampled] = particles[particle_index];
			++num_sampled;
		}
	}
	
	for(size_t i = 0; i < num_predict_particles; ++i)
	{
		//select particle to predict forward
		temp_particles[i].x += vel_x * dt + x_predict_dist(gen);
		temp_particles[i].y += vel_y * dt + y_predict_dist(gen);
		temp_particles[i].theta = last_theta + theta_fog_dist(gen);
	}

	//copy over the particles
	for(size_t i = 0; i < temp_particles.size(); ++i)
	{
		particles[i] = temp_particles[i];
	}
}
void Localizer::createParticles(int64_t curr_utime)
{
	//create particles from last generation of particles
	if(last_utime != 0)
	{
		createPredictionParticles(curr_utime);
	}

	//add the gps distribution
	random_device rd;
	mt19937 gen(rd());
	size_t i = ((last_utime == 0)? 0: num_predict_particles);
	for(; i < particles.size(); ++i)
	{
		particles[i].x = last_coord.first + x_gps_dist(gen);
		particles[i].y = last_coord.second + y_gps_dist(gen);
		particles[i].theta = last_theta + theta_fog_dist(gen);
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
