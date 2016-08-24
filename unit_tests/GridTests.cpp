#include "../src/GridMap.hpp"
#include <cassert>
#include <iostream>

using namespace std;

class GridTester : private GridMap
{
public:
	GridTester(double mnx, double mxx, double mny, double mxy, double ss):
		GridMap(mnx, mxx, mny, mxy, ss)
	{
	}

	void testCoordTransform(double test_x, double test_y, size_t idx)
	{
		assert(convertToGridCoords(test_x, test_y) == idx);
	}
};

int main()
{
	GridTester test(-1, 2, -1, 2, 1);
	size_t idx = 0;
	for(double x = -1.0; x < 2.0; x += .01)
	{
		for(double y = -1.0; y < 2.0; y += .01)
		{
			if(x < 0.0)
			{
				if(y < 0.0)
				{
					idx = 0;
				}
				else if(y < 1.0)
				{
					idx = 3;
				}
				else if(y < 2.0)
				{
					idx = 6;
				}
			}
			else if(x < 1.0)
			{
				if(y < 0.0)
				{
					idx = 1;
				}
				else if(y < 1.0)
				{
					idx = 4;
				}
				else if(y < 2.0)
				{
					idx = 7;
				}
			}
			else if(x < 2.0)
			{
				if(y < 0.0)
				{
					idx = 2;
				}
				else if(y < 1.0)
				{
					idx = 5;
				}
				else if(y < 2.0)
				{
					idx = 8;
				}
			}

			test.testCoordTransform(x, y, idx);
			cout << "test of " << x << ' ' << y << " passed" << endl;
		}
	}

	test.testCoordTransform(0,0, 4);
	test.testCoordTransform (-1, -1, 0);
	test.testCoordTransform(-0.5, -1, 0);
}

