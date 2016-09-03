#include "MapDrawer.hpp"
#include <thread>

using namespace std;

void MapDrawer::startDrawThread()
{
	thread draw_thread(&MapDrawer::startDraw, this);
	draw_thread.detach();
}

void MapDrawer::switchMap(const GridMap& nmap)
{
	unique_lock<mutex> map_loc(map_mut);
	map = nmap;
}

void MapDrawer::startDraw()
{
	sf::RenderWindow window(sf::VideoMode(800,600), "Map");
	while(window.isOpen())
	{
		//check the window's events
		sf::Event event;
		while(window.pollEvent(event))
		{
			if(event.type == sf::Event::Closed)
			{
				window.close();
			}
		}

		//clear the window
		window.clear(sf::Color::White);
		drawMap(window);
		
	}
}
//0,0 = (400, 300)
void MapDrawer::drawMap(sf::RenderWindow & win)
{
	unique_lock<mutex> map_lock(map_mut);

	size_t num_cols = map.getCellsPerRow();
	size_t num_rows = map.getNumCells()/num_cols;

	//calculate row and column for 0,0
	size_t origin_loc = map.convertToGridCoords(0,0);
	size_t origin_col =	origin_loc % num_cols;
	size_t origin_row = origin_loc / num_cols;

	//calculate the square pixel size
	size_t pix_per_square = std::min(700/num_cols, 500/num_rows);

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

		int min_x = 300 + (col_num - origin_col) * pix_per_square;
		int min_y = 400 + (row_num - origin_row) * pix_per_square;

		square[0].position = sf::Vector2f(min_x, min_y);
		square[1].position = sf::Vector2f(min_x + pix_per_square, min_y);
		square[2].position = sf::Vector2f(min_x + pix_per_square, min_y + pix_per_square);
		square[3].position = sf::Vector2f(min_x, min_y + pix_per_square);
		
		square[0].color = sf::Color(0,0,0, map[i]);
		square[1].color = sf::Color(0,0,0, map[i]);
		square[2].color = sf::Color(0,0,0, map[i]);
		square[3].color = sf::Color(0,0,0, map[i]);
	}

	win.draw(grid);
	win.display();
}
