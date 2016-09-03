#ifndef __UMA_MAP_DRAWER_HPP__
#define __UMA_MAP_DRAWER_HPP__

#include "GridMap.hpp"
#include <mutex>

class MapDrawer
{
public:
	void startDrawThread();
	void switchMap(const GridMap& nmap);
	void drawLoop();

private:
	std::mutex map_mut;
	GridMap map;
};
#endif
