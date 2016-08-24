#ifndef __GRID_MAP_HPP__
#define __GRID_MAP_HPP__

#include <vector>
#include <cstdint>

class GridMap
{
	const double FULL_INCREMENT = 1.0;
	const double EMPTY_DECREMENT = -1.0;
public:
	GridMap(double mnx, double mxx, double mny, double mxy, double ss);

	uint8_t at(double x, double y) const;

	//accesses the vector 
	uint8_t& at(double x, double y);

protected:
	//helper function
	std::size_t convertToGridCoords(double x, double y) const;

private:
	//actual map containing the proabilities 
	std::vector<uint8_t> map;

	//extents of the map in x and y
	double max_x;
	double max_y;
	double min_x;
	double min_y;
	
	//square size in meters
	double square_size_meters;

	//helper variables
	std::size_t cells_per_row;
};
#endif
