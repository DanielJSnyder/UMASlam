#include "../lcmtypes/slam_pc_t.hpp"
#include "../src/Constants.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>

using namespace std;
using namespace SLAM::LCM;

class Printer
{
public:
void handlePC(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const slam_pc_t * pc)
{
	for(size_t i = 0; i < pc->cloud.size(); ++i)
	{
		for(size_t j = 0; j < pc->cloud[i].size(); ++j)
		{
			cout << pc->cloud[i][j].x << '\t' << pc->cloud[i][j].y << '\t' << pc->cloud[i][j].z << endl;
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
