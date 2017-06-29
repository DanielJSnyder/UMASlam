#include "SLAM.hpp"
#include "MapDrawer.hpp"
#include "Constants.hpp"
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
    l.subscribe(SLAM_MAP_CHANNEL, &MapDrawer::handleMap, &drawer);
    drawer.startDrawThread();
    cout << "started draw thread" << endl;
    while(1)
    {
      l.handle();
    }
  }
  else
  {
    Slam s;
    s.run();
  }

}
