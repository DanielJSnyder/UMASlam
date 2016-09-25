#include "../src/PointCloud.hpp"
#include "../src/Constants.hpp"

#include <lcm/lcm-cpp.hpp>

int main()
{
	PointCloudMaker pcm;
	lcm::LCM lcm;
	lcm.subscribe(LASER_SCAN_CHANNEL, &PointCloudMaker::handleLaserScan, &pcm);
	lcm.subscribe(SERVO_CHANNEL, &PointCloudMaker::handleServo, &pcm);

	while(1)
	{
		lcm.handle();
	}
	
}
