#include "../src/SLAM.hpp"
#include "../src/MapDrawer.hpp"
#include "../src/Constants.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char ** argv)
{
	Slam s;
	
	bool drawing = (argc == 2 && string(argv[1]) == "-d");
	if(drawing)
	{
		std::thread slam_thread(&Slam::run, &s);
		cout << "started slam thread" << endl;
		MapDrawer drawer;
		lcm::LCM l;
		l.subscribe(SLAM_STATE_CHANNEL, &MapDrawer::handleState, &drawer);
		drawer.startDrawThread();
		cout << "started draw thread" << endl;
		while(1)
		{
			drawer.switchMap(s.getMap());
			l.handle();
		}
	}
	else
	{
		s.run();
	}

}
