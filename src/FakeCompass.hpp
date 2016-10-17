#ifndef __SLAM_FAKE_COMPASS_HPP__
#define __SLAM_FAKE_COMPASS_HPP__

#include "../lcmtypes/gps_t.hpp"
#include "../lcmtypes/fog_t.hpp"
#include "CoordTransformer.hpp"
#include <vector>
#include <lcm/lcm-cpp.hpp>

class FakeCompass
{
public:
	double getNorthLocation();

	void addGPS(const common::LCM::types::gps_t & gps_data);

	void addFOG(const common::LCM::types::fog_t & fog_data);

	size_t getNumCoords() const 
	{
		return xy_coords.size();
	}

	double getDistFromOrigin() const;

private:
	CoordTransformer coord_transformer;
	std::vector<std::pair<double, double> > xy_coords;
	std::vector<double> angles;
};

#endif
