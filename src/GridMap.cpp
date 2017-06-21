#include "GridMap.hpp"
#include "Constants.hpp"
#include <cmath>
#include <iostream>

using namespace std;
using namespace SLAM::LCM;

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
	map((abs(mnx)+ abs(mxx))/ss * (abs(mny) + abs(mxy))/ss, INITIAL_MAP_VALUE),//intialize the map to be all grey
	max_x(mxx),
	max_y(mxy),
	min_x(mnx),
	min_y(mny),
	square_size_meters(ss)
{
	cells_per_row = (abs(mnx) + abs(mxx))/square_size_meters;
}

GridMap::GridMap(double mnx, double mxx, double mny, double mxy, double ss, std::vector<int16_t> map_in):
	map((abs(mnx)+ abs(mxx))/ss * (abs(mny) + abs(mxy))/ss, INITIAL_MAP_VALUE),//intialize the map to be all grey
	max_x(mxx),
	max_y(mxy),
	min_x(mnx),
	min_y(mny),
	square_size_meters(ss)
{
	cells_per_row = (abs(mnx) + abs(mxx))/square_size_meters;
  map = map_in;
  /*
  cout << "max_x: " << max_x << endl;
  cout << "max_y: " << max_y << endl;
  cout << "min_x: " << min_x << endl;
  cout << "min_y: " << min_y << endl;

  cout << "square_size_meters: " << square_size_meters << endl;
  cout << "cells_per_row: " << cells_per_row << endl;

  cout << "MAP SIZE: " << map.size() << endl;
  */
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

void GridMap::resetMap()
{
	map.assign(map.size(), INITIAL_MAP_VALUE);
}

void GridMap::publishMap(int64_t utime, string channel) {
  // Copy map into an LCM object
  // along with all relevant data
	SLAM::LCM::slam_map_t publish_map;
  publish_map.utime = utime;
  publish_map.map_size = map.size();
  publish_map.max_x = max_x;
  publish_map.max_y = max_y;
  publish_map.min_x = min_x;
  publish_map.min_y = min_y;
  publish_map.map_size = map.size();
  publish_map.square_size_meters = square_size_meters;
  publish_map.cells_per_row = cells_per_row;
  publish_map.map = map;

  lcm::LCM lcm;
  lcm.publish(channel, &publish_map);
}
