#include "MapDrawer.hpp"
#include <iostream>
#include <thread>
#include "Constants.hpp"

#define WINDOW_HEIGHT 1600.0f
#define WINDOW_WIDTH 1200.0f

using namespace std;
using namespace common::LCM::types;

void MapDrawer::startDrawThread()
{
	cerr << "Starting Draw Thread" << endl;
	thread draw_thread(&MapDrawer::startDraw, this);
	draw_thread.detach();
}

void MapDrawer::switchMap(const GridMap& nmap)
{
	unique_lock<mutex> map_loc(map_mut);
	map = nmap;
}

void MapDrawer::addPose(const SLAM::Pose& pose)
{
	poses.push_back(pose);
}

void MapDrawer::startDraw()
{
	sf::RenderWindow window(sf::VideoMode(WINDOW_HEIGHT, WINDOW_WIDTH), "Map");
	sf::View view = window.getDefaultView();
	view.zoom(2.0f);
	double current_zoom = 1.0;
	while(window.isOpen())
	{
		for(int i = 0; i < 10; ++i)
		{
			this_thread::sleep_for(std::chrono::milliseconds(100));
			view.zoom(1.0/current_zoom);
			//check the window's events
			sf::Event event;
			while(window.pollEvent(event))
			{
				switch(event.type)
				{
					case sf::Event::Closed:		
						window.close();
						break;

					case sf::Event::MouseWheelMoved:
						if(event.mouseWheel.delta < 0)			current_zoom = max(current_zoom - .1, .125);
						else if(event.mouseWheel.delta > 0)	current_zoom = min(current_zoom + .1, 4.0);
						break;

					case sf::Event::MouseButtonPressed:
					{
						sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
						sf::Vector2f worldPos = window.mapPixelToCoords(pixelPos);
						view.setCenter(worldPos);
					}
						break;

					default:					break;
				}
			}
			view.zoom(current_zoom);
			window.setView(view);
			window.display();
		}
		//clear the window
		window.clear(sf::Color::White);
		drawMap(window);
		drawPoses(window);
		drawBoat(window);

		window.setView(view);
		window.display();
	}
}

void MapDrawer::drawMap(sf::RenderWindow & win)
{
	unique_lock<mutex> map_lock(map_mut);

	size_t num_cols = map.getCellsPerRow();
	if(num_cols == 0)
	{
		win.display();
		return;
	}
	//size_t num_rows = map.getNumCells()/num_cols;

	//calculate row and column for 0,0
	size_t origin_loc = map.convertToGridCoords(0,0);
	size_t origin_col =	origin_loc % num_cols;
	size_t origin_row = origin_loc / num_cols;

	sf::VertexArray grid;
	grid.setPrimitiveType(sf::Quads);
	grid.resize(map.getNumCells() * 4);
	size_t row_num = 0;
	size_t col_num = 0;
	for(size_t i = 0; i < map.getNumCells(); ++i, ++col_num)
	{
		//greater than, or <=?
		if(num_cols < col_num)
		{
			col_num -= num_cols;
			++row_num;
		}
		sf::Vertex * square = &grid[i*4];

		int halfx = WINDOW_HEIGHT/2.0;
		int halfy = WINDOW_WIDTH/2.0;
		int min_x = halfx +(col_num - origin_col) * PIX_PER_SQUARE;
		int min_y = halfy +(row_num - origin_row) * PIX_PER_SQUARE;

		square[0].position = sf::Vector2f(min_x, min_y);
		square[1].position = sf::Vector2f(min_x + PIX_PER_SQUARE, min_y);
		square[2].position = sf::Vector2f(min_x + PIX_PER_SQUARE, min_y + PIX_PER_SQUARE);
		square[3].position = sf::Vector2f(min_x, min_y + PIX_PER_SQUARE);

		square[0].color = sf::Color(0,0,0, map[i]);
		square[1].color = sf::Color(0,0,0, map[i]);
		square[2].color = sf::Color(0,0,0, map[i]);
		square[3].color = sf::Color(0,0,0, map[i]);
	}
	win.draw(grid);
}

pair<double, double> MapDrawer::convertToPixelCoords(double x, double y)
{
	double origin_x = WINDOW_HEIGHT/2.0;
	double origin_y = WINDOW_WIDTH/2.0;

	double pix_per_meter = static_cast<double>(PIX_PER_SQUARE)/map.getSquareSize();

	x *= pix_per_meter;
	y *= pix_per_meter;
	x += origin_x;
	y += origin_y;
	return pair<double, double>(x,y);
}

void MapDrawer::handleState(const lcm::ReceiveBuffer * rbuf, const string & chan, const state_t * state)
{
	SLAM::Pose p;
	p.x = state->x;
	p.y = state->y;
	p.theta = state->yaw;
	addPose(p);
}

void MapDrawer::handleMap(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const SLAM::LCM::slam_map_t * slam_map) {
  GridMap gridmap(slam_map->min_x, slam_map->max_x, slam_map->min_y, slam_map->max_y, slam_map->square_size_meters, slam_map->map);
  map = gridmap; 
}

void MapDrawer::drawBoat(sf::RenderWindow & win)
{
	if(poses.size() == 0)
		return;

	sf::VertexArray boat;
	boat.setPrimitiveType(sf::Triangles);
	boat.resize(3);

	double forward_dx = cos(poses.back().theta);
	double forward_dy = sin(poses.back().theta);

	pair<double, double> coord2 = convertToPixelCoords(forward_dx + poses.back().x,
													   forward_dy + poses.back().y);
	pair<double, double> coord1 = convertToPixelCoords(-forward_dx - forward_dy/2.0 + poses.back().x,
													   -forward_dy + forward_dx/2.0 + poses.back().y);
	pair<double, double> coord3 = convertToPixelCoords(-forward_dx  + forward_dy/2.0 + poses.back().x,
													   -forward_dy - forward_dx/2.0 + poses.back().y);
	
	boat[0].position = sf::Vector2f(coord1.first, coord1.second);
	boat[0].color = sf::Color::Blue;
	boat[1].position = sf::Vector2f(coord2.first, coord2.second);
	boat[1].color = sf::Color::Red;
	boat[2].position = sf::Vector2f(coord3.first, coord3.second);
	boat[2].color = sf::Color::Blue;

	win.draw(boat);
}

void MapDrawer::drawPoses(sf::RenderWindow & win)
{
	sf::VertexArray pose_line;
	pose_line.setPrimitiveType(sf::LinesStrip);
	pose_line.resize(poses.size());
	for(size_t i = 0/*std::max(0, (int)(poses.size()) - NUM_POSES_TO_DRAW)*/;
		i < poses.size(); ++i)
	{
		SLAM::Pose p = poses[i];
		pair<double, double> coords = convertToPixelCoords(p.x, p.y);
		pose_line[i].position = sf::Vector2f(coords.first, coords.second);
		pose_line[i].color = sf::Color::Red;
	}
	win.draw(pose_line);

}
