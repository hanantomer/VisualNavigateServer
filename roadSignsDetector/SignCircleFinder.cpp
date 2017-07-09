
#include "opencv2/imgproc.hpp"
#include "global.h"
#include "SignCircleFinder.h"
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <ctime>


using namespace cv;

SignCircleFinder::Point::Point(SimplePoint &point)
{
	x = point.x;
	y = point.y;
	m_x = 0;
	m_y = 0;
}

SignCircleFinder::Point::Point() {}

SignCircleFinder::SignCircleFinder()
{
}

SignCircleFinder::~SignCircleFinder()
{
}

vector<SimpleCircle> SignCircleFinder::FindCircleSigns(Mat &whiteCloseToBlackFiltered, Mat &whiteCloseToBlack, Mat &whiteCloseToBlackExtended, Mat &maskCombinedBlack, Mat &source)
{
	clock_t begin = clock();

	vector<SimpleCircle> circleCandidates;
	
	std::map<int, bool*> whiteRowsFiltered = utils::GetMatRows(whiteCloseToBlackFiltered);

	std::map<int, bool*> whiteRows = utils::GetMatRows(whiteCloseToBlackFiltered);

	clock_t end1 = clock();
	double elapsed_secs = double(end1 - begin) / CLOCKS_PER_SEC;
	cout << "end1 elapsed seconds:" << elapsed_secs;

	vector<SimpleCircle> squareSimpleCircles = 
		FindCircles(whiteRowsFiltered, whiteRows, whiteCloseToBlackFiltered, whiteCloseToBlack, whiteCloseToBlackExtended);

	for (size_t i = 0; i < squareSimpleCircles.size(); i++)
	{
		StoreCircle(squareSimpleCircles.at(i), circleCandidates);
	}

	for (size_t i = 0; i < circleCandidates.size(); i++)
	{
		int colStart = circleCandidates.at(i).center.x - circleCandidates.at(i).radius;

		int rowStart = circleCandidates.at(i).center.y - circleCandidates.at(i).radius;

		circleCandidates.at(i) =
			CalibrateCircle(circleCandidates.at(i), maskCombinedBlack, source);
	}

	clock_t end2 = clock();
	double elapsed_secs2 = double(end2 - begin) / CLOCKS_PER_SEC;

	cout<<"FindCircleSigns elapsed seconds:"<<elapsed_secs2;

	if(doFilter)
		return circleCandidates;
	
	return circleCandidates;
	
}

vector<SimpleCircle> SignCircleFinder::FindCircles(map<int, bool*> &whiteRowsFiltered, map<int, bool*> &whiteRows, Mat whiteCloseToBlackFiltered, Mat whiteCloseToBlack, Mat whiteCloseToBlackExtended)
{

	vector<SimpleCircle> circleCandidates;

	typedef map<int, bool*> matRows;

	std::map<int, bool*> usedPoints;

	for(matRows::iterator itFiltered = whiteRowsFiltered.begin(); itFiltered != whiteRowsFiltered.end(); itFiltered++)
	{
		int y = itFiltered->first;

		SimpleCircle simpleCircle = 
			GetLineCircle(y, itFiltered->second, whiteRows, whiteCloseToBlack, whiteCloseToBlackExtended, usedPoints);

		if (simpleCircle.radius > 0)
		{
			StoreCircle(simpleCircle, circleCandidates);
		}
	}

	return circleCandidates;
}



SimpleCircle SignCircleFinder::GetLineCircle(int  y, bool* xArrFiltered, map<int, bool*> &whiteRows, Mat whiteCloseToBlack, Mat whiteCloseToBlackExtended, map<int, bool*> &usedPoints)
{

	SimpleCircle simpleCircle;
	simpleCircle.radius = 0;

	int minXDistance = minCircleRadius;
	int maxXDistance = maxCircleRadius * 2;

	int cols = whiteCloseToBlack.cols;

	bool tupleFound = false;

	for (size_t col = 0; col < cols; col++)
	{
		bool x = xArrFiltered[col];

		if (x) {
			
			if (usedPoints[y] && usedPoints[y][col])
			{
				usedCount++;
				continue;
			}

			SimplePoint *tuple = new SimplePoint[3];

			tuple[0].x = col;
			tuple[0].y = y;
			tuple[1].x = 0;
			tuple[1].y = 0;
			tuple[2].x = 0;
			tuple[2].y = 0;

			// find next x
			int nextXStart = col + minXDistance;
			int nextXEnd = col + maxXDistance;
			if (nextXEnd >= cols)
				nextXEnd = cols - 1;
			int nextX = 0;

			bool *xArr[3];
			for (int i = -1; i < 2; i++)
			{
				if (y + i > 0 && y + i < whiteRows.size())
				{
					xArr[i + 1] = whiteRows.at(y + i);
				}
				else 
				{
					xArr[i] = 0;
				}
			}


			while (nextX <= nextXEnd && nextX > 0);
			{
				nextX = GetTupleNextX(nextXStart, nextXEnd, xArr);
				nextXStart++;

				if (nextX == 0)
				{
					continue;
				}
				

				tuple[1].y = y;
				tuple[1].x = nextX;


				// find next y
				int minYDistance = /* 2* */ minCircleRadius - ((tuple[1].x - tuple[0].x) / 2);
				int maxYDistance = 30 + 2 * maxCircleRadius - ((tuple[1].x - tuple[0].x) / 2);

				
				vector<SimplePoint> nextYPoints = GetTupleNextYPoints(tuple[0].x, tuple[1].x, y + minYDistance, y + maxYDistance, whiteRows);

				for (size_t p = 0; p < nextYPoints.size(); p++)
				{
					tuple[2] = nextYPoints.at(p);

					simpleCircle = GetCircleFromTuple(tuple, whiteCloseToBlack.cols, whiteCloseToBlack.rows);

					if (simpleCircle.radius > 0)
					{
						int circlePoints = utils::GetCirclePoints(simpleCircle, whiteCloseToBlackExtended);
					
						if (circlePoints >= min_cicle_points)
						{
							simpleCircle.points = circlePoints;
							return simpleCircle;
						}
						
						int y1 = tuple[2].y;
						if (!usedPoints[y1])
						{
							usedPoints[y1] = new bool[cols];
							std::fill_n(usedPoints[y1], cols, false);
						}

						bool* arr = usedPoints[y1];
						arr[tuple[2].x] = true;
					}
				}
			}
		}
	}

	return simpleCircle;
}

int SignCircleFinder::GetTupleNextX(int nextXStart, int nextXEnd, bool *xArr[3])
{
	int nextX = 0;
	for (size_t i = 0; i < 3; i++)
	{
		if (xArr[i])
		{
			for (size_t j = nextXEnd; j > nextXStart; j--)
			{
				if (xArr[i][j])
				{
					nextX = j;
					return nextX;
				}
			}
		}
	}

	return nextX;
}

vector<SimplePoint>  SignCircleFinder::GetTupleNextYPoints( int nextXStart, int nextXEnd, int minY, int maxY, map<int, bool*> &whiteRows)
{
	vector<SimplePoint> pointArr;
	//for (size_t k = maxY; k >= minY && k < whiteRows.size(); k--)
	for (size_t k = minY; k < maxY && k < whiteRows.size(); k++)
	{
		bool* nextYXArr = whiteRows.at(k);
		for (size_t x = nextXEnd; x >= nextXStart; x--)
		{
			if (nextYXArr[x])
			{
				SimplePoint sp;
				sp.x = x;
				sp.y = k;

				pointArr.push_back(sp);
			}
		}
	}

	return pointArr;
}


bool SignCircleFinder::IsPerpendicular(Point *pt1, Point *pt2, Point *pt3)
// Check the given point are perpendicular to x or y axis 
{
	double yDelta_a = pt2->y - pt1->y;
	double xDelta_a = pt2->x - pt1->x;
	double yDelta_b = pt3->y - pt2->y;
	double xDelta_b = pt3->x - pt2->x;


	//	TRACE(" yDelta_a: %f xDelta_a: %f \n",yDelta_a,xDelta_a);
	//	TRACE(" yDelta_b: %f xDelta_b: %f \n",yDelta_b,xDelta_b);

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

SimpleCircle SignCircleFinder::GetCircleFromTuple(SimplePoint *tuple, int cols, int rows)
{
	int circle_distance_from_edge = 8;

	Point *pt1 = new Point(tuple[0]);
	Point *pt2 = new Point(tuple[1]);
	Point *pt3 = new Point(tuple[2]);

	SimpleCircle simpleCircle;
	simpleCircle.radius = -1;


	if (!this->IsPerpendicular(pt1, pt2, pt3))			simpleCircle = this->CalcCircle(pt1, pt2, pt3);
	else if (!this->IsPerpendicular(pt1, pt3, pt2))		simpleCircle = this->CalcCircle(pt1, pt3, pt2);
	else if (!this->IsPerpendicular(pt2, pt1, pt3))		simpleCircle = this->CalcCircle(pt2, pt1, pt3);
	else if (!this->IsPerpendicular(pt2, pt3, pt1))		simpleCircle = this->CalcCircle(pt2, pt3, pt1);
	else if (!this->IsPerpendicular(pt3, pt2, pt1))		simpleCircle = this->CalcCircle(pt3, pt2, pt1);
	else if (!this->IsPerpendicular(pt3, pt1, pt2))		simpleCircle = this->CalcCircle(pt3, pt1, pt2);

	delete pt1;				delete pt2;				delete pt3;

	if (simpleCircle.radius < 0															||
		simpleCircle.center.x < 0														||
		simpleCircle.radius < minCircleRadius											||
		simpleCircle.radius > maxCircleRadius											||
		simpleCircle.center.x - simpleCircle.radius - circle_distance_from_edge < 0		||
		simpleCircle.center.x + simpleCircle.radius + circle_distance_from_edge > cols	||
		simpleCircle.center.y - simpleCircle.radius - circle_distance_from_edge < 0		||
		simpleCircle.center.y + simpleCircle.radius + circle_distance_from_edge > rows) {

		simpleCircle.radius = 0;
	}

	return simpleCircle;
}


SimpleCircle SignCircleFinder::CalcCircle(Point *pt1, Point *pt2, Point *pt3)
{
	SimpleCircle simpleCircle;

	simpleCircle.radius = -1;
	//simpleCircle.count = 1;


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

/*
vector<SimpleCircle> SignCircleFinder::FindFrequentCircles(vector<SimpleCircle> &circleCandidates)
{
	vector<SimpleCircle> frequentSimpleCircles;


	for (size_t i = 0; i < circleCandidates.size(); i++)
	{
		if (circleCandidates.at(i).count > min_frequency_threshold )
		{
			frequentSimpleCircles.push_back(circleCandidates.at(i));
		}
	}

	return frequentSimpleCircles;
}
*/

bool SignCircleFinder::CircleIsMatched(SimpleCircle circle1, SimpleCircle circle2)
{
	if (abs(circle1.center.x - circle2.center.x) > circle_match_threshold)
		return false;

	if (abs(circle1.center.y - circle2.center.y) > circle_match_threshold)
		return false;

	if (abs(circle1.radius - circle2.radius) > circle_match_threshold)
		return false;


	return true;
}

void SignCircleFinder::StoreCircle(SimpleCircle circle, vector<SimpleCircle> &circleCandidates)
{
	if (!doMatchNearCircles)
	{
		circleCandidates.push_back(circle);
		return;
	}

	if (circleCandidates.size() == 0) {
		circleCandidates.push_back(circle);
	}
	else
	{
		bool matchFound = false;

		for (size_t i = 0; i < circleCandidates.size(); i++)
		{
			if (utils::CircleContains(circle, circleCandidates.at(i)))
			{
				circleCandidates.at(i) = circle;
				matchFound = true;
			}
			else if (utils::CircleContains(circleCandidates.at(i), circle))
			{
				matchFound = true;
			}
			else if (CircleIsMatched(circle, circleCandidates.at(i)))
			{
				//ConsolidateCircle(circle, circleCandidates.at(i));
				//circleCandidates.at(i).count++;
				circleCandidates.at(i) = ChooseCircleWithMorePoints(circle, circleCandidates.at(i));
				matchFound = true;
			}
		}

		if (!matchFound)
			circleCandidates.push_back(circle);
	}
}

void SignCircleFinder::ConsolidateCircle(SimpleCircle &sourceCircle, SimpleCircle &targetCircle)
{
	targetCircle.radius =
		(sourceCircle.radius + targetCircle.radius * targetCircle.points) / (targetCircle.points + 1);

	targetCircle.center.x =
		(sourceCircle.center.x + targetCircle.center.x * targetCircle.points) / (targetCircle.points + 1);

	targetCircle.center.y =
		(sourceCircle.center.y + targetCircle.center.y * targetCircle.points) / (targetCircle.points + 1);
}

SimpleCircle SignCircleFinder::ChooseCircleWithMorePoints(SimpleCircle &sourceCircle, SimpleCircle &targetCircle)
{
	return sourceCircle.points > targetCircle.points ? sourceCircle : targetCircle;
}

SimpleCircle SignCircleFinder::CalibrateCircle(SimpleCircle circle, Mat &maskCombinedBlack, Mat &source)
{
	if (!doCalibrate)
		return circle;

	int left = max(0, circle.center.x - position_calibration_threshold);
	int right = circle.center.x + position_calibration_threshold;
	int top = max(0, circle.center.y - position_calibration_threshold);
	int bottom = circle.center.y + position_calibration_threshold;

	SimpleCircle calibratedCircle = 
		utils::CalibrateCircle(circle, maskCombinedBlack, source, SIGN_FRAME_RELATIVE_WIDTH);

	return calibratedCircle;
}








