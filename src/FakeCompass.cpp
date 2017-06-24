#include "FakeCompass.hpp" 
#include "Utilities.hpp"
#include <algorithm>

using namespace std;
using namespace common::LCM::types;

double FakeCompass::getNorthLocation()
{
  double total_fog = std::accumulate(angles.begin(), angles.end(), 0.0);
  // Finds a fairly accurate heading of the boat assuming that it goes mostly in one direction
  double average_fog = total_fog/static_cast<double>(angles.size());
  
  //calculate the final angle of travel
  double end_angle = atan2(xy_coords.back().second, xy_coords.back().first);

  return (average_fog - end_angle);
}

// Returns the difference between the initial angle and the angle of the current position from
// the GPS' north axis. Used to initialize FOG's initial angle
double FakeCompass::getNorthLocation(double initial_theta)
{
  // End angle is calculated relative to GPS north
  double end_angle = atan2(xy_coords.back().second, xy_coords.back().first);
  // Returns an offset used to zero out the FOG at north
  return (initial_theta - end_angle);
}

void FakeCompass::addGPS(const gps_t & gps_data)
{
  if(!coord_transformer.isInitialized())
  {
    coord_transformer.initialize(gps_data.latitude, gps_data.longitude);
  }
  xy_coords.push_back(coord_transformer.transform(gps_data.latitude, gps_data.longitude));
}

void FakeCompass::addFOG(const fog_t & fog_data)
{
  angles.push_back(DEG_TO_RAD(fog_data.data));
}

double FakeCompass::getDistFromOrigin() const
{
  double x = xy_coords.back().first;
  double y = xy_coords.back().second;
  // Dan didn't take the square root here for some reason, presumably an
  // optimization since it doesn't really matter as long as
  // ORIGIN_DIST_BEFORE_REINITIALIZATION is adjusted accordingly
  return (x*x + y*y);
}
