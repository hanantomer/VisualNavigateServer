#include "global.h"
#pragma once
class CircleFromTriangleCalculator
{
public:
	SimpleCircle CircleFromTriangleCalculator::GetCircleFromTriangle(SimplePoint *triangle, int matNumCols, int matNumRows);

	struct Point
	{
		Point();
		Point(SimplePoint&);
		int x;
		int y;
		double m_x;
		double m_y;
	};


private:
	const int circle_distance_from_edge = 8;
	SimpleCircle CalcCircle(Point *pt1, Point *pt2, Point *pt3);
	bool IsPerpendicular(Point *pt1, Point *pt2, Point *pt3);
};
