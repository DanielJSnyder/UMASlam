#ifndef __GRID_MAP_HPP__
#define __GRID_MAP_HPP__

#include <vector>
#include <cstdint>
#include <string>
#include "../lcmtypes/slam_map_t.hpp"
#include <lcm/lcm-cpp.hpp>

//coord system
// ^ x
// |
// |
// |___>	y
class GridMap
{
public:
	GridMap();

	GridMap(double mnx, double mxx, double mny, double mxy, double ss);

	GridMap(const GridMap& gm);
	const GridMap & operator=(const GridMap & gm);

	std::size_t convertToGridCoords(double x, double y) const
	{
		//map is filled in this manner
		/*
			index 0 is the cell who's bottom left corner is at min_x, min_y
			index 1 is the cell who's bottom left_corner is at min_x + square_size_meters, min_y

		*/

		//calculate the distance from the bottom left of the map
		double tx = x - min_x;
		double ty = y - min_y;

		std::size_t x_idx = tx/square_size_meters;
		std::size_t y_idx = ty/square_size_meters;

		return (x_idx + y_idx * cells_per_row);
	}

	int16_t at(double x, double y) const
	{
		std::size_t idx = convertToGridCoords(x,y);
		return map[idx];
	}

	//accesses the vector 
	int16_t& at(double x, double y)
	{
		std::size_t idx = convertToGridCoords(x,y);
		return map[idx];
	}

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

	const int16_t& operator[](std::size_t idx) const 
	{
		return map[idx];
	}

	int16_t& operator[](std::size_t idx)
	{
		return map[idx];
	}

	void resetMap();

  void publishMap(int64_t utime, std::string channel);
private:
	//actual map containing the probabilities 
	//idx 0 is bottom left (-y, -x)
	std::vector<int16_t> map;

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
