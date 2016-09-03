#ifndef __UMA_MAP_DRAWER_HPP__
#define __UMA_MAP_DRAWER_HPP__

#include "GridMap.hpp"
#include <SFML/Graphics.hpp>
#include <mutex>

class MapDrawer
{
public:
	void startDrawThread();
	void switchMap(const GridMap& nmap);
	void startDraw();
	void drawMap(sf::RenderWindow & win);

private:
	std::mutex map_mut;
	GridMap map;
};
#endif
