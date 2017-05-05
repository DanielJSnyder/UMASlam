#include "../lcmtypes/slam_pc_t.hpp"
#include "../src/Constants.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <sys/time.h>

using namespace std;
using namespace SLAM::LCM;

class Printer
{
public:
void handlePC(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const slam_pc_t * pc)
{
	cout << "New PointCloud of size: " << pc->cloud.size() << endl;
	for(size_t i = 0; i < pc->cloud.size(); ++i)
	{
		cout << "Scan line " << i << "\nsize: " << pc->cloud[i].scan_line.size() << endl;
		for(size_t j = 0; j < pc->cloud[i].scan_line.size(); ++j)
		{
			cout << pc->cloud[i].scan_line[j].x << '\t' << pc->cloud[i].scan_line[j].y << '\t' << pc->cloud[i].scan_line[j].z << endl;
		}
	}
	cout << endl;
}
};

int main()
{
	Printer p;
	lcm::LCM lcm;
	lcm.subscribe(SLAM_POINT_CLOUD_CHANNEL, &Printer::handlePC, &p);
  
	while(1)
	{
		lcm.handle();
	}
}
