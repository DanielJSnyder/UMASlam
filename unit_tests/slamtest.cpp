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
	bool drawing = (argc == 2 && string(argv[1]) == "-d");
	if(drawing)
	{
	  Slam s;
		std::thread slam_thread(&Slam::run, &s);
		cout << "started slam thread" << endl;
		MapDrawer drawer;
		lcm::LCM l;
		l.subscribe(SLAM_STATE_CHANNEL, &MapDrawer::handleState, &drawer);
    l.subscribe(SLAM_PC_MAP_CHANNEL, &MapDrawer::handleMap, &drawer);
		drawer.startDrawThread();
		cout << "started draw thread" << endl;
		while(1)
		{
			//drawer.switchMap(s.getMap());
			l.handle();
		}
	}
	else
	{
    // s.run() has its own loop, so this will just create and 
    // start SLAM again whenever SLAM gets killed. At the time of
    // this comment, SLAM should only get stopped by an LCM message
    // in SLAM_RESET_CHANNEL
    while(1)
    {
	    Slam s;
      s.run();
    }
	}

}
