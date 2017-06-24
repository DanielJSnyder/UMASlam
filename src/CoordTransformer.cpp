#include "CoordTransformer.hpp"
#include <cmath>

CoordTransformer::CoordTransformer() : initialized(false),
                     origin_lat(0.0),
                     origin_lon(0.0),
                     origin_earth_radius(0.0)
                     {}

void CoordTransformer::initialize(double initial_lat, double initial_lon)
{
  //set the lat lon of the origin
  origin_lat = initial_lat;
  origin_lon = initial_lon;

  //find the geocentric radius of the origin
  double term1 = EARTH_RADIUS_METERS_EQUATOR * std::cos(origin_lat *M_PI/180.0);
  double term2 = EARTH_RADIUS_METERS_POLE * std::sin(origin_lat *M_PI/180.0);
  double term3 = EARTH_RADIUS_METERS_EQUATOR * term1;
  double term4 = EARTH_RADIUS_METERS_POLE * term2;
  origin_earth_radius = sqrt(((term3*term3) + (term4*term4))/(term1*term1 + term2*term2));

  initialized = true;
}

XYCoord CoordTransformer::transform(double lat, double lon)
{
  //These values are in degrees
  double d_lat = lat - origin_lat;
  double d_lon = lon - origin_lon;

  //Change the degrees to radians
  d_lat *= M_PI/180.0;
  d_lon *= M_PI/180.0;

  //This is different from how april does it.  April assumes the world has no
  //curvature at the local point.  This does not make that assumption
  //Also April has different radii for ew and ns directions, which this does not
  double delta_x = d_lat * origin_earth_radius;
  double delta_y = d_lon * origin_earth_radius * std::cos(origin_lat * M_PI/180.0);

  return XYCoord(delta_x, delta_y);
}
