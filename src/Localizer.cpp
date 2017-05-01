#include "Localizer.hpp"
#include "Constants.hpp"
#include "../lcmtypes/state_t.hpp"
#include "Utilities.hpp"
#include "../lcmtypes/particles_t.hpp"
#include <queue>
#include <algorithm>
#include <utility>

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
	gps_dist(0, gps_sigma),
	theta_fog_dist(0, fog_sigma),
	x_predict_dist(0, X_PREDICTION_SIGMA),
	y_predict_dist(0, Y_PREDICTION_SIGMA),
	num_predict_particles(num_particles *predict_percent),
	last_utime(0),
	fog_initialized(false)
{
  last_imu_data.utime = 0;
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

void Localizer::handleIMUData(const lcm::ReceiveBuffer * rbuf,
							  const string & chan,
							  const imu_t * imu_data)
{
  double last_x_accel = last_imu_data.vdot;
  double last_y_accel = last_imu_data.udot;

  double x_accel = imu_data->vdot;
  double y_accel = imu_data->udot;

  // Width of trapezoid
  int64_t time_diff = imu_data->utime - last_utime;

  // Trapezoidal Riemann sum between last two accelerations to find velocity
  vel.x += time_diff * ((last_x_accel + x_accel) / 2);
  vel.y += time_diff * ((last_y_accel + y_accel) / 2);
}

void Localizer::updateInternals(int64_t utime)
{
	previous_gen_coord = last_coord;
	last_utime = utime;
}

void Localizer::weightParticles(const slam_pc_t & pc)
{
	// clear the old likelihoods and create the particles that will be weighted
	createParticles(pc.utime);
	clearLikelihoods();

	// weight the Particles based on the sensor data
	weightParticlesWithGPS(last_coord);
	weightParticlesWithFOG(last_theta);
	weightParticlesWithCloud(pc);

	//set pose
	setPose(pc.utime);
	publishPose();

	//clean up internals
	updateInternals(pc.utime);
}

void Localizer::weightParticlesWithGPS(const pair<double, double> & GPS_basis)
{
	constexpr double square_root_2pi = 2.50662827463100050241577;//obtained from wolfram alpha
	const double gaussian_coeff = 1.0/(square_root_2pi * gps_dist.stddev());
	
	for(Particle & p : particles)
	{
		double dx = GPS_basis.first - p.x;
		double dy = GPS_basis.second - p.y;
		double distance_from_last_gps_point_m = sqrt(dx*dx + dy*dy);

		//error in gps is gaussian
		double numerator = distance_from_last_gps_point_m - gps_dist.mean();
		double denom = 2.0 * gps_dist.stddev();
		double gps_likelihood = gaussian_coeff * exp(-(numerator * numerator)/(denom));

		//Don't need to bound the likelihoods,
		//since the gaussian already makes them between 0 and 1
		//so just add it to the particle
		p.likelihood += gps_likelihood * GPS_LIKELIHOOD_COEFFICIENT;
	}
}

void Localizer::weightParticlesWithFOG(const double last_theta)
{
	constexpr double square_root_2pi = 2.50662827463100050241577;//obtained from wolfram alpha
	const double gaussian_coeff = 1.0/(square_root_2pi * theta_fog_dist.stddev());
	
	for(Particle & p : particles)
	{
		double delta_theta = p.theta - last_theta;
		//error in fog is gaussian
		double numerator = delta_theta - theta_fog_dist.mean();
		double denom = 2.0 * theta_fog_dist.stddev();
		double fog_likelihood = gaussian_coeff * exp(-(numerator * numerator)/(denom));

		//Don't need to bound the likelihoods,
		//since the gaussian already makes them between 0 and 1
		//so just add it to the particle
		p.likelihood += fog_likelihood * FOG_LIKELIHOOD_COEFFICIENT;
	}
}

void Localizer::weightParticlesWithCloud(const slam_pc_t & pc)
{
	vector<double> temp_likelihoods(particles.size(), 0);
	size_t num_hits = 0;
	for(size_t i = 0; i < particles.size(); ++i)
	{
		Particle & p = particles[i];
		double curr_particle_likelihood = 0;
		SLAM::Pose particle_pose(p.x, p.y,p.theta, 0);

		//for each scan line and each end point do hit or miss
		SLAM::logDebugMsg("point_cloud size: " + to_string(pc.cloud.size()) + "\n", 1);
		for(size_t i = 0; i < pc.cloud.size(); ++i)
		{
			SLAM::logDebugMsg("scan size: " + to_string(pc.cloud[i].scan_line.size()) + "\n", 1);
			for(size_t j = 0; j < pc.cloud[i].scan_line.size(); ++j)
			{
				if(pc.cloud[i].hit[j] == 0 )
				{
					continue;
				}
				num_hits++;
				double x = pc.cloud[i].scan_line[j].x;
				double y = pc.cloud[i].scan_line[j].y;
				double z = pc.cloud[i].scan_line[j].z;
				double angle = atan2(y,x);

				if(abs(angle * 180.0/M_PI ) > LIDAR_MAP_RANGE_DEG)
				{
					continue;
				}
			
				SLAM::rotateIntoGlobalCoordsInPlace(x,y,z, particle_pose);
				
				//just do simple hit or miss as lidar sigma < 30mm from data sheet
				if(map.at(x, y) > HIT_THRESHOLD)//if the square is considered full, add to likelihood
				{ 
					curr_particle_likelihood += HIT_LIKELIHOOD_INC_VALUE; 
				}
				else if(map.at(x,y) < MISS_THRESHOLD)
				{
					curr_particle_likelihood += MISS_LIKELIHOOD_DEC_VALUE;
				}
			}
		}
		if(num_hits<= MINIMUM_LIDAR_HITS_TO_WEIGHT)//if the scan is empty, exit early
		{
			return;
		}
		temp_likelihoods[i] = curr_particle_likelihood;
	}

	num_hits /= particles.size();

	double min_laser_likelihood = MISS_LIKELIHOOD_DEC_VALUE * num_hits;
	double max_laser_likelihood = HIT_LIKELIHOOD_INC_VALUE * num_hits;
	boundLikelihoods(temp_likelihoods, min_laser_likelihood, max_laser_likelihood);

	for(size_t i = 0; i < temp_likelihoods.size(); ++i)
	{
		particles[i].likelihood += temp_likelihoods[i] * LASER_LIKELIHOOD_COEFFICIENT;
	}
}

void Localizer::clearLikelihoods()
{
	for(Particle & p : particles)
	{
		p.likelihood = 0;
	}
}

void Localizer::boundLikelihoods(vector<double> & likelihoods, double min_likelihood, double max_likelihood) const
{
	double max_for_division = max_likelihood - min_likelihood;
	for(double & d : likelihoods)
	{
		d = (d - min_likelihood)/max_for_division;
		d = min(1.0, max(0.0, d));
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

	//move the particles into a vector for analysis
	vector<pair<Particle, double> > averaging_particles(NUM_AVERAGE_PARTICLES);
	for(size_t i = 0; i < NUM_AVERAGE_PARTICLES && !part_queue.empty(); ++i)
	{
		averaging_particles[i].first = part_queue.top();
		averaging_particles[i].second = 0.0;
		part_queue.pop();
	}

	for(size_t i = 0; i < averaging_particles.size(); ++i)
	{
		for(size_t j = i+1; j < averaging_particles.size(); ++j)
		{
			double dx = averaging_particles[i].first.x - averaging_particles[j].first.x;
			double dy = averaging_particles[i].first.y - averaging_particles[j].first.y;
			double dist = sqrt(dx*dx + dy*dy);

			averaging_particles[i].second += dist;
			averaging_particles[j].second += dist;
		}
	}
	
	sort(averaging_particles.begin(), averaging_particles.end(), [](const pair<Particle,double> &  a, const pair<Particle,double> &  b) { return a.second < b.second;});

	for(size_t i = 0; i < NUM_OUTLIERS_TO_REMOVE; ++i)
	{
		averaging_particles.pop_back();
	}

	double total_x = 0; 
	double total_y = 0;
	double total_theta = 0;

	for(size_t i = 0; i < averaging_particles.size(); ++i)
	{
		total_x += averaging_particles[i].first.x;
		total_y += averaging_particles[i].first.y;
		total_theta += averaging_particles[i].first.theta;
	}

	last_pose.x = total_x/(static_cast<double>(averaging_particles.size()));
	last_pose.y = total_y/(static_cast<double>(averaging_particles.size()));
	last_pose.theta = total_theta/(static_cast<double>(averaging_particles.size()));
	last_pose.utime = utime;
}

void Localizer::publishPose() const
{
	state_t pub_state;
	pub_state.utime = last_pose.utime;
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

	//get the maximum likelihood
	double sum_of_likelihoods = 0;
	for(Particle & p: particles)
	{
		sum_of_likelihoods += p.likelihood;
	}

	uniform_real_distribution<double> initialization_distr(0, sum_of_likelihoods/num_predict_particles);
	random_device rd;
	mt19937 gen(rd());

	//select the particles that will be predicted forward
	size_t particle_index = 0;
	size_t num_sampled = 0;
	double total_likelihood = particles[0].likelihood;
	std::vector<Particle> temp_particles(num_predict_particles);

	for(double curr_likelihood = initialization_distr(gen);
		num_sampled < num_predict_particles && particle_index < particles.size();
		curr_likelihood += sum_of_likelihoods/num_predict_particles)
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
	

  double dx;
  double dy;
  //IMU goes here, can replace dx and dy with data from the IMU
  if(USE_IMU) 
  {
    dx = vel.x;  
    dy = vel.y;
  }
  else
  {
    //calculate prediction params
    dx = (last_coord.first - previous_gen_coord.first);
    dy = (last_coord.second - previous_gen_coord.second);
  }

	for(size_t i = 0; i < num_predict_particles; ++i)
	{
		temp_particles[i].x += dx +  x_predict_dist(gen);
		temp_particles[i].y += dy + y_predict_dist(gen);
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

	//add the gps and fog distribution
	uniform_real_distribution<double> rotational_distr(-M_PI, M_PI);
	random_device rd;
	mt19937 gen(rd());
	size_t i = ((last_utime == 0)? 0: num_predict_particles);
	for(; i < particles.size(); ++i)
	{
		double dist_from_mean = gps_dist(gen);
		double rotation_angle = rotational_distr(gen);
		particles[i].x = last_coord.first + dist_from_mean * std::cos(rotation_angle);
		particles[i].y = last_coord.second + dist_from_mean * std::sin(rotation_angle);
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
