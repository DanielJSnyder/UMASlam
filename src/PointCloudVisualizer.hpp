#include <lcm/lcm-cpp.hpp>
#include <SFML/Graphics.hpp>
#include "../lcmtypes/slam_pc_t.hpp"

class PointCloudVisualizer
{
public:
	void handlePointCloud(const lcm::ReceiveBuffer * rbuf,
						  const std::string & chan,
						  const SLAM::LCM::slam_pc_t * pc);

	void drawCloud(sf::RenderWindow & win);

	void run();

private:
	SLAM::LCM::slam_pc_t cloud;
};
