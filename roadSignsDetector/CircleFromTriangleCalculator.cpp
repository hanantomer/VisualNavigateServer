#include "CircleFromTriangleCalculator.h";
#include "global.h";

CircleFromTriangleCalculator::Point::Point(SimplePoint &point)
{
	x = point.x;
	y = point.y;
	m_x = 0;
	m_y = 0;
}

CircleFromTriangleCalculator::Point::Point() {}


SimpleCircle CircleFromTriangleCalculator::GetCircleFromTriangle(SimplePoint *triangle, int matNumCols, int matNumRows)
{
	Point *pt1 = new Point(triangle[0]);
	Point *pt2 = new Point(triangle[1]);
	Point *pt3 = new Point(triangle[2]);

	SimpleCircle simpleCircle;
	simpleCircle.radius = -1;


	if (!this->IsPerpendicular(pt1, pt2, pt3))			simpleCircle = this->CalcCircle(pt1, pt2, pt3);
	else if (!this->IsPerpendicular(pt1, pt3, pt2))		simpleCircle = this->CalcCircle(pt1, pt3, pt2);
	else if (!this->IsPerpendicular(pt2, pt1, pt3))		simpleCircle = this->CalcCircle(pt2, pt1, pt3);
	else if (!this->IsPerpendicular(pt2, pt3, pt1))		simpleCircle = this->CalcCircle(pt2, pt3, pt1);
	else if (!this->IsPerpendicular(pt3, pt2, pt1))		simpleCircle = this->CalcCircle(pt3, pt2, pt1);
	else if (!this->IsPerpendicular(pt3, pt1, pt2))		simpleCircle = this->CalcCircle(pt3, pt1, pt2);

	delete pt1;				delete pt2;				delete pt3;

	if (simpleCircle.radius < 0 ||
		simpleCircle.center.x < 0 ||
		simpleCircle.radius < minCircleRadius ||
		simpleCircle.radius > maxCircleRadius ||
		simpleCircle.center.x - simpleCircle.radius - circle_distance_from_edge < 0 ||
		simpleCircle.center.x + simpleCircle.radius + circle_distance_from_edge > matNumCols ||
		simpleCircle.center.y - simpleCircle.radius - circle_distance_from_edge < 0 ||
		simpleCircle.center.y + simpleCircle.radius + circle_distance_from_edge > matNumRows) {

		simpleCircle.radius = 0;
	}

	return simpleCircle;
}

SimpleCircle CircleFromTriangleCalculator::CalcCircle(Point *pt1, Point *pt2, Point *pt3)
{
	SimpleCircle simpleCircle;

	simpleCircle.radius = -1;

	double yDelta_a = pt2->y - pt1->y;
	double xDelta_a = pt2->x - pt1->x;
	double yDelta_b = pt3->y - pt2->y;
	double xDelta_b = pt3->x - pt2->x;

	if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001) {
		//TRACE("Calc cirlce \n");
		simpleCircle.center.x = (int)(0.5*(pt2->x + pt3->x));
		simpleCircle.center.y = (int)(0.5*(pt1->y + pt2->y));
		//this->m_Center.m_z = pt1->z();

		double xDistance = abs((double)simpleCircle.center.x - (double)pt1->x);
		double yDistance = abs((double)simpleCircle.center.y - (double)pt1->y);
		double distance = sqrt(pow(xDistance, 2) + pow(yDistance, 2));
		simpleCircle.radius = (int)distance;

		return simpleCircle;
	}

	// IsPerpendicular() assure that xDelta(s) are not zero
	double aSlope = yDelta_a / xDelta_a; // 
	double bSlope = yDelta_b / xDelta_b;
	if (fabs(aSlope - bSlope) <= 0.000000001) {	// checking whether the given points are colinear. 	
												//TRACE("The three pts are colinear\n");
		return simpleCircle;
	}

	// calc center
	double a = aSlope*bSlope*(pt1->y - pt3->y);
	double b = bSlope*(pt1->x + pt2->x);
	double c = aSlope*(pt2->x + pt3->x);
	double d = (2 * (bSlope - aSlope));

	simpleCircle.center.x = (int)((a + b - c) / d);

	simpleCircle.center.y = (int)(-1 * (simpleCircle.center.x - (pt1->x + pt2->x) / 2) / aSlope + (pt1->y + pt2->y) / 2);
	//this->m_Center.m_z = pt1->m_z;

	double xDistance = abs((double)simpleCircle.center.x - (double)pt1->x);
	double yDistance = abs((double)simpleCircle.center.y - (double)pt1->y);
	double distance = sqrt(pow(xDistance, 2) + pow(yDistance, 2));
	simpleCircle.radius = (int)distance;

	return simpleCircle;
}

bool CircleFromTriangleCalculator::IsPerpendicular(Point *pt1, Point *pt2, Point *pt3)
// Check the given point are perpendicular to x or y axis 
{
	double yDelta_a = pt2->y - pt1->y;
	double xDelta_a = pt2->x - pt1->x;
	double yDelta_b = pt3->y - pt2->y;
	double xDelta_b = pt3->x - pt2->x;

	// checking whether the line of the two pts are vertical
	if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001) {
		//		TRACE("The points are pependicular and parallel to x-y axis\n");
		return false;
	}

	if (fabs(yDelta_a) <= 0.0000001) {
		//		TRACE(" A line of two point are perpendicular to x-axis 1\n");
		return true;
	}
	else if (fabs(yDelta_b) <= 0.0000001) {
		//		TRACE(" A line of two point are perpendicular to x-axis 2\n");
		return true;
	}
	else if (fabs(xDelta_a) <= 0.000000001) {
		//		TRACE(" A line of two point are perpendicular to y-axis 1\n");
		return true;
	}
	else if (fabs(xDelta_b) <= 0.000000001) {
		//		TRACE(" A line of two point are perpendicular to y-axis 2\n");
		return true;
	}
	else return false;
}







