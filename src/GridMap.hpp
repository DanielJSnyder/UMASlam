#ifndef __GRID_MAP_HPP__
#define __GRID_MAP_HPP__

#include <vector>
#include <cstdint>


//coord system
// ^ x
// |
// |
// |___>	y
class GridMap
{
	const double FULL_INCREMENT = 1.0;
	const double EMPTY_DECREMENT = -1.0;
public:
	GridMap();

	GridMap(double mnx, double mxx, double mny, double mxy, double ss);

	GridMap(const GridMap& gm);
	const GridMap & operator=(const GridMap & gm);

	uint8_t at(double x, double y) const;

	//accesses the vector 
	uint8_t& at(double x, double y);

	//helper function
	std::size_t convertToGridCoords(double x, double y) const;

	double getMaxX() const
	{
		return max_x;
	}
	double getMinX() const
	{
		return min_x;
	}
	double getMaxY() const
	{
		return max_y;
	}
	double getMinY() const
	{
		return min_y;
	}
	double getSquareSize() const
	{
		return square_size_meters;
	}
	std::size_t getCellsPerRow() const 
	{
		return cells_per_row;
	}
	std::size_t getNumCells() const
	{
		return map.size();
	}

	const uint8_t& operator[](std::size_t idx) const;
private:
	//actual map containing the probabilities 
	//idx 0 is bottom left (-y, -x)
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
