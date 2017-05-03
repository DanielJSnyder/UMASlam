#include "../src/SLAM.hpp"
#include "../src/MapDrawer.hpp"
#include "../src/Constants.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <string>

// For ctrl-c handling
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

bool loop = true;

void exitHandler(int s) {
  loop = false;
}

int main(int argc, char ** argv)
{
  struct sigaction sigIntHandler;
  
  sigIntHandler.sa_handler = exitHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

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
		while(loop)
		{
			drawer.switchMap(s.getMap());
			l.handle();
		}
	}
	else
	{
		std::thread slam_thread(&Slam::run, &s);
    while(loop) { 
      cout << ""; 
    }

    //Continue to run slam until ctrl-c
    //cout << "HIT CTRL C" << endl;
    s.stop();

	}

}
