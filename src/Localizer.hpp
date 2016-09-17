#ifndef __SLAM_LOCALIZER_HPP__
#define __SLAM_LOCALIZER_HPP__

#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "../lcmtypes/laser_t.hpp"
#include "GridMap.hpp"
#include "Pose.hpp"
#include "CoordTransformer.hpp"
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <vector>
#include <random>

struct Particle
{
	double x;
	double y;
	double theta;
	double likelihood;

	Particle();
};

class Localizer
{
public:
	Localizer(int num_particles);
	Localizer(int num_particles, double gps_sigma, double fog_sigma);

	void handleGPSData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::gps_t * gps_data);

	void handleFOGData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::fog_t * fog_data);

	void handleLaserScan(const lcm::ReceiveBuffer * rbuf,
						 const std::string & chan,
						 const common::LCM::types::laser_t * laser_scan);

	SLAM::Pose getPose() const;

	void updateMap(const GridMap & new_map);
private:

	void fillParticles(const common::LCM::types::gps_t & gps_data);
	void fillParticles(double theta);

	void weightParticles(const common::LCM::types::laser_t & laser_scan);

	void boundLikelihoods();
	void setPose();
	void publishPose() const;

	GridMap map;
	std::vector<Particle> particles;
	SLAM::Pose last_pose;

	CoordTransformer coord_transformer;

	std::normal_distribution<> x_gps_dist;
	std::normal_distribution<> y_gps_dist;
	std::normal_distribution<> theta_fog_dist;
	double last_theta;
	double initial_theta;
	const double LIKELIHOOD_INCREMENT_VALUE = 1.0;//arbitrary for now
	bool fog_initialized;
};

#endif
