#include "../src/SLAM.hpp"
#include "../src/MapDrawer.hpp"
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;

int main()
{
	Slam s;
	std::thread slam_thread(&Slam::run, &s);
	cout << "started slam thread" << endl;
	
	MapDrawer drawer;
	lcm::LCM l;
	l.subscribe("SLAM_STATE", &MapDrawer::handleState, &drawer);
	drawer.startDrawThread();
	cout << "started draw thread" << endl;

	while(1)
	{
		drawer.switchMap(s.getMap());
			l.handle();
	}
}
