#ifndef __SLAM_LOCALIZER_HPP__
#define __SLAM_LOCALIZER_HPP__

#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "../lcmtypes/slam_pc_t.hpp"
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

	bool operator<(const Particle & po) const
	{
		return po.likelihood < likelihood;
	}
};

struct ParticleComparer
{
	bool operator()(const Particle & p1, const Particle & p2)
	{
		return p2.likelihood < p1.likelihood;
	}
};

class Localizer
{
public:
	Localizer(int num_particles, double predict_percent);
	Localizer(int num_particles, double predict_percent, double gps_sigma, double fog_sigma);

	void handleGPSData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::gps_t * gps_data);

	void handleFOGData(const lcm::ReceiveBuffer * rbuf,
					   const std::string & chan,
					   const common::LCM::types::fog_t * fog_data);

	void handlePointCloud(const lcm::ReceiveBuffer * rbuf,
						  const std::string & chan,
						  const SLAM::LCM::slam_pc_t * pc);

	SLAM::Pose getPose() const;

	void updateMap(const GridMap & new_map);

	void reset();
	void reinitializeFOG(double new_initial_fog);

	double getFogInitialization() const;

private:
	void createPredictionParticles(int64_t curr_utime);
	void createParticles(int64_t utime);

	void weightParticles(const SLAM::LCM::slam_pc_t & pc);

	void boundLikelihoods();
	void setPose(int64_t utime);
	void publishPose() const;
	void publishParticles() const;

	GridMap map;
	std::vector<Particle> particles;
	SLAM::Pose last_pose;

	std::pair<double, double> last_coord;
	double last_theta;

	CoordTransformer coord_transformer;

	std::normal_distribution<> x_gps_dist;
	std::normal_distribution<> y_gps_dist;
	std::normal_distribution<> theta_fog_dist;
	std::normal_distribution<> x_predict_dist;
	std::normal_distribution<> y_predict_dist;
	double initial_theta;

	//variables for predicting particles forward
	size_t num_predict_particles;
	int64_t last_utime;
	std::pair<double, double> previous_gen_coord;

	bool fog_initialized;
};

#endif
