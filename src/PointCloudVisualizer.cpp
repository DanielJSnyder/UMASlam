#include "PointCloudVisualizer.hpp"
#include "Constants.hpp"

using namespace std;
using namespace SLAM::LCM;

void PointCloudVisualizer::handlePointCloud(const lcm::ReceiveBuffer * rbuf,
											const string & chan,
											const slam_pc_t * pc)
{
	cloud = *pc;
}

void PointCloudVisualizer::run()
{
	sf::RenderWindow window(sf::VideoMode(800, 800), "Point Cloud");
	lcm::LCM my_lcm;
	my_lcm.subscribe(SLAM_POINT_CLOUD_CHANNEL, &PointCloudVisualizer::handlePointCloud, this);

	while(window.isOpen())
	{
		my_lcm.handleTimeout(20);
		window.clear(sf::Color::White);
		drawCloud(window);
		window.display();
	}
}

void PointCloudVisualizer::drawCloud(sf::RenderWindow & win)
{
	sf::Vertex virt_split [] = {sf::Vertex(sf::Vector2f(400, 0), sf::Color::Black), sf::Vertex(sf::Vector2f(400, 800), sf::Color::Black)};
	sf::Vertex horz_split [] = {sf::Vertex(sf::Vector2f(0, 400), sf::Color::Black), sf::Vertex(sf::Vector2f(800, 400), sf::Color::Black)};
	win.draw(virt_split, 2, sf::Lines);
	win.draw(horz_split, 2, sf::Lines);
	sf::View xy_plane;
	sf::View yz_plane;
	sf::View xz_plane;

	xy_plane.setViewport(sf::FloatRect(0.0,0.0, 0.5, 0.5));
	yz_plane.setViewport(sf::FloatRect(0.5,0, 0.5, 0.5));
	xz_plane.setViewport(sf::FloatRect(0, 0.5, 0.5, 0.5));

	sf::Vertex right_axis[] = 
	{
		sf::Vertex(sf::Vector2f(500, 500), sf::Color::Blue),
		sf::Vertex(sf::Vector2f(550, 500), sf::Color::Blue)
	};

	sf::Vertex down_axis[] = 
	{
		sf::Vertex(sf::Vector2f(500, 500), sf::Color::Green),
		sf::Vertex(sf::Vector2f(500, 550), sf::Color::Green)
	};

	const sf::Color x_color = sf::Color::Red;
	const sf::Color y_color = sf::Color::Blue;
	const sf::Color z_color = sf::Color::Green;
	//draw the axis
	win.setView(xy_plane);
	right_axis[0].color = x_color;right_axis[1].color = x_color;
	down_axis[0].color = y_color;down_axis[1].color = y_color;
	win.draw(right_axis, 2, sf::Lines);
	win.draw(down_axis, 2, sf::Lines);

	win.setView(yz_plane);
	right_axis[0].color = y_color;right_axis[1].color = y_color;
	down_axis[0].color = z_color;down_axis[1].color = z_color;
	win.draw(right_axis, 2, sf::Lines);
	win.draw(down_axis, 2, sf::Lines);

	win.setView(xz_plane);
	right_axis[0].color = x_color;right_axis[1].color = x_color;
	win.draw(right_axis, 2, sf::Lines);
	win.draw(down_axis, 2, sf::Lines);

	for(auto scan_line : cloud.cloud)
	{
		for(int i = 0; i < scan_line.scan_size; ++i)
		{
			point3D_t point = scan_line.scan_line[i];
			sf::CircleShape p;
			if(scan_line.hit[i] == 1)
			{
				p.setFillColor(sf::Color::Black);
			}
			if(scan_line.hit[i] == 0)
			{
				p.setFillColor(sf::Color::Magenta);
			}
			p.setRadius(3);

			win.setView(xy_plane);
			p.setPosition(point.x * 20 + 500, point.y * 20 + 500);
			win.draw(p);

			win.setView(yz_plane);
			p.setPosition(point.y * 20 + 500, point.z * 20 + 500);
			win.draw(p);

			win.setView(xz_plane);
			p.setPosition(point.x * 20 + 500, point.z * 20 + 500);
			win.draw(p);
		}
	}
	win.setView(win.getDefaultView());
}

int main()
{
	PointCloudVisualizer pcv;
	pcv.run();
}
