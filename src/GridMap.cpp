#include "GridMap.hpp"
#include <cmath>

using namespace std;

GridMap::GridMap(double mnx, double mxx, double mny, double mxy, double ss):
	map((abs(mnx)+ abs(mxx))/ss * (abs(mny) + abs(mxy))/ss, 128),//intialize the map to be all grey
	max_x(mxx),
	max_y(mxy),
	min_x(mnx),
	min_y(mny),
	square_size_meters(ss)
{
	cells_per_row = (abs(mnx) + abs(mxx))/square_size_meters;
}

uint8_t GridMap::at(double x, double y) const
{
	size_t idx = convertToGridCoords(x,y);
	return map[idx];
}

uint8_t& GridMap::at(double x, double y)
{
	size_t idx = convertToGridCoords(x,y);
	return map[idx];
}

size_t GridMap::convertToGridCoords(double x, double y) const
{
	//map is filled in this manner
	/*
		index 0 is the cell who's bottom left corner is at min_x, min_y
		index 1 is the cell who's bottom left_corner is at min_x + square_size_meters, min_y

	*/

	//calculate the distance from the bottom left of the map
	double tx = x - min_x;
	double ty = y - min_y;

	size_t x_idx = tx/square_size_meters;
	size_t y_idx = ty/square_size_meters;

	return (x_idx + y_idx * cells_per_row);
}

const uint8_t& GridMap::operator[] (size_t idx) const
{
	return map[idx];
}
