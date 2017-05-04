#include "../src/Mapper.hpp"
#include "../src/MapDrawer.hpp"
#include "../src/Constants.hpp"

#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <iostream>

using namespace std;

int main()
{
	//initialize a giant map
	Mapper mapper(-30, 30, -30, 30, .25);

	lcm::LCM lcm;
	lcm.subscribe(SLAM_POINT_CLOUD_CHANNEL, &Mapper::handlePointCloud, &mapper);
	lcm.subscribe(STATE_CHANNEL, &Mapper::handleState, &mapper);
	
	MapDrawer drawer;
	drawer.startDrawThread();

	while(0 == lcm.handle())
	{
		//handle any backlog of events before drawing
		while(lcm.handleTimeout(10) > 0)
		{
		}
		drawer.switchMap(mapper.getMapCopy());
		this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}
