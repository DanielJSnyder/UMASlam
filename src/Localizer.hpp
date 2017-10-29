#ifndef __SLAM_LOCALIZER_HPP__
#define __SLAM_LOCALIZER_HPP__

#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "../lcmtypes/imu_t.hpp"
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

struct Velocity {
  double x;
  double y;
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

  void weightParticles(const SLAM::LCM::slam_pc_t * pc);
  void weightParticlesWithGPS(const std::pair<double, double> & GPS_basis);
  void weightParticlesWithFOG(const double last_theta);
  void weightParticlesWithCloud(const SLAM::LCM::slam_pc_t & pc);

  void boundLikelihoods(std::vector<double> & likelihoods, double min_likelihood, double max_likelihood) const;
  void setPose(int64_t utime);
  void publishPose() const;
  void publishParticles() const;
  void clearLikelihoods();
  void updateInternals(int64_t utime);

  GridMap map;
  std::vector<Particle> particles;
  SLAM::Pose last_pose;

  std::pair<double, double> last_coord;
  double last_theta;

  CoordTransformer coord_transformer;

  std::normal_distribution<> gps_dist;
  std::normal_distribution<> theta_fog_dist;
  std::normal_distribution<> x_predict_dist;
  std::normal_distribution<> y_predict_dist;
  double initial_theta;

  //variables for predicting particles forward
  size_t num_predict_particles;
  int64_t last_utime;
  std::pair<double, double> previous_gen_coord;

  bool fog_initialized;

  // Stores the most recently received message's utime
  int64_t current_utime;
};

#endif
