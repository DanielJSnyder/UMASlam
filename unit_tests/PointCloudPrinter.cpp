#include "../lcmtypes/point_cloud_t.hpp"
#include "../src/Constants.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>

using namespace std;
using namespace common::LCM::types;

class Printer
{
public:
void handlePC(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const point_cloud_t * pc)
{
	for(size_t i = 0; i < pc->x.size(); ++i)
	{
		for(size_t j = 0; j < pc->x[i].size(); ++j)
		{
			cout << pc->x[i][j] << '\t' << pc->y[i][j] << '\t' << pc->z[i][j] << endl;
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
