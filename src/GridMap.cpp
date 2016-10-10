#include "GridMap.hpp"
#include <cmath>

using namespace std;

GridMap::GridMap() : 
	map(0,0),
	max_x(0),
	max_y(0),
	min_x(0),
	min_y(0),
	square_size_meters(0),
	cells_per_row(0)
{
}

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

GridMap::GridMap(const GridMap & gm)
{
	*this=gm;
}

const GridMap& GridMap::operator=(const GridMap & gm)
{
	map = gm.map;
	max_x = gm.max_x;
	min_x = gm.min_x;
	max_y = gm.max_y;
	min_y = gm.min_y;
	square_size_meters = gm.square_size_meters;
	cells_per_row = gm.cells_per_row;

	return *this;
}
