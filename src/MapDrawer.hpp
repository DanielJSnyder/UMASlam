#ifndef __UMA_MAP_DRAWER_HPP__
#define __UMA_MAP_DRAWER_HPP__

#include "GridMap.hpp"
#include "../lcmtypes/state_t.hpp"
#include <SFML/Graphics.hpp>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <vector>
#include <utility>
#include "Pose.hpp"

constexpr double WINDOW_HEIGHT = 1600.0;
constexpr double WINDOW_WIDTH = 1200.0;

constexpr size_t PIX_PER_SQUARE = 4;

class MapDrawer
{
public:
	void startDrawThread();
	void switchMap(const GridMap& nmap);
	void addPose(const SLAM::Pose& pose);
	void startDraw();
	void drawMap(sf::RenderWindow & win);
	void drawPoses(sf::RenderWindow & win);

	void handleState(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const common::LCM::types::state_t * state);

private:
	std::pair<double, double> convertToPixelCoords(double x, double y);

	std::mutex map_mut;
	GridMap map;
	std::vector<SLAM::Pose> poses;

};
#endif
