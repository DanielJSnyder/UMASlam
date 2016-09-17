#ifndef __COORD_TRANSFORMER_HPP__
#define __COORD_TRANSFORMER_HPP__
#include <utility>

constexpr int EARTH_RADIUS_METERS_EQUATOR = 6378137;//from Wikipedia
constexpr int EARTH_RADIUS_METERS_POLE = 6356752.3;//from Wikipedia

typedef std::pair<double, double> XYCoord;
class CoordTransformer
{
public:

	CoordTransformer();
	/*
		Purpose: sets the position that will be 0,0 to be the  coordinates provided
	*/
	void initialize(double initial_lat, double initial_lon);
	/*
		Purpose: Returns whether the coordinates that all transformations
		are based on have been provided.  This is equivalent to calling the initialize()
		function.
	*/
	bool isInitialized() const { return initialized;}
	
	/*
		Purpose: takes the latitude, and longitude and returns the associated x,y coordinates
		Notes:  This makes the assumption that the boat does not move very far from the
		origin relative to the size of the globe.  Also makes the assumption you are not
		close to one of the poles.
		This also assumes you are at sealevel;
	*/
	XYCoord transform(double lat, double lon);
private:
	bool initialized;

	double origin_lat;
	double origin_lon;
	double origin_earth_radius;
};
#endif
