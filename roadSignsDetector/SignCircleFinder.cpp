
#include "opencv2/imgproc.hpp"
#include "global.h"
#include "SignCircleFinder.h"
#include "CircleFromTriangleCalculator.h"
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <ctime>

using namespace cv;


SignCircleFinder::SignCircleFinder(Mat source, string sourceImageFileName)
{
	this->sourceImageFileName = sourceImageFileName;
	this->source = source;
}

SignCircleFinder::~SignCircleFinder()
{
}

vector<SimpleCircle> SignCircleFinder::Run()
{
	this->CreateExtendedMatrixes();
	
	vector<SimpleCircle> circleCandidates;

	vector<SimpleCircle> squareSimpleCircles = this->FindCircles();

	for (size_t i = 0; i < squareSimpleCircles.size(); i++)
	{
		StoreCircle(squareSimpleCircles.at(i), circleCandidates);
	}

	for (size_t i = 0; i < circleCandidates.size(); i++)
	{
		int colStart = circleCandidates.at(i).center.x - circleCandidates.at(i).radius;

		int rowStart = circleCandidates.at(i).center.y - circleCandidates.at(i).radius;

		if (doCalibrate)
		{
			circleCandidates.at(i) =
				utils::CalibrateCircle(circleCandidates.at(i), maskCombinedBlack, source, SIGN_FRAME_RELATIVE_WIDTH);
		}
	}


	return circleCandidates;
}

void SignCircleFinder::CreateExtendedMatrixes()
{

	utils::GetEdgePoints(this->source, this->whiteCloseToBlack, this->whiteCloseToBlackExtended, this->maskCombinedBlack);

	this->whiteCloseToBlackFiltered = this->whiteCloseToBlack.clone();

	utils::FilterByDensity(this->whiteCloseToBlackFiltered, this->filterSquareSize);

	this->whiteRowsFiltered = utils::GetMatRows(this->whiteCloseToBlackFiltered);

	this->whiteRows = utils::GetMatRows(this->whiteCloseToBlackFiltered);

	utils::SaveFile(string("whiteCloseToBlack.png"), this->whiteCloseToBlack, this->sourceImageFileName);
	utils::SaveFile(string("whiteCloseToBlackExtended.png"), this->whiteCloseToBlackExtended, this->sourceImageFileName);
	utils::SaveFile(string("maskCombinedBlack.png"), this->maskCombinedBlack, this->sourceImageFileName);
	utils::SaveFile(string("whiteCloseToBlackFiltered.png"), this->whiteCloseToBlackFiltered, this->sourceImageFileName);
}

vector<SimpleCircle> SignCircleFinder::FindCircles()
{
	vector<SimpleCircle> circleCandidates;

	typedef map<int, bool*> matRows;

	std::map<int, bool*> usedPoints;

	matRows::iterator itFiltered = whiteRowsFiltered.begin();

	for (int i = 0; i < whiteRowsFiltered.size() - 5; i++)
	{
		itFiltered++;

		int row = itFiltered->first;

		SimpleCircle simpleCircle = 
			GetRowCircle(row, itFiltered->second, whiteRows, whiteCloseToBlack, whiteCloseToBlackExtended, usedPoints);

		if (simpleCircle.radius > 0 && simpleCircle.points > 0)
		{
			StoreCircle(simpleCircle, circleCandidates);
		}
	}

	return circleCandidates;
}

SimpleCircle SignCircleFinder::GetRowCircle(int  row, bool* xArrFiltered, map<int, bool*> &whiteRows, Mat whiteCloseToBlack, Mat whiteCloseToBlackExtended, map<int, bool*> &usedPoints)
{
	CircleFromTriangleCalculator *circleFromTriangleCalculator = new CircleFromTriangleCalculator();


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
			
			if (usedPoints[row] && usedPoints[row][col])
			{
				usedCount++;
				continue;
			}

			SimplePoint *tuple = new SimplePoint[3];

			tuple[0].x = col;
			tuple[0].y = row;
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
				if (row + i > 0 && row + i < whiteRows.size())
				{
					xArr[i + 1] = whiteRows.at(row + i);
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
				

				tuple[1].y = row;
				tuple[1].x = nextX;


				// find next y
				int minYDistance = /* 2* */ minCircleRadius - ((tuple[1].x - tuple[0].x) / 2);
				int maxYDistance = 30 + 2 * maxCircleRadius - ((tuple[1].x - tuple[0].x) / 2);

				
				vector<SimplePoint> nextYPoints = GetTupleNextYPoints(tuple[0].x, tuple[1].x, row + minYDistance, row + maxYDistance, whiteRows);

				for (size_t p = 0; p < nextYPoints.size(); p++)
				{
					tuple[2] = nextYPoints.at(p);

					simpleCircle = 
						circleFromTriangleCalculator->GetCircleFromTriangle(tuple, whiteCloseToBlack.cols, whiteCloseToBlack.rows);

					if (simpleCircle.radius > 0)
					{
						int circlePoints = utils::GetCirclePoints(simpleCircle, whiteCloseToBlackExtended);

						simpleCircle.points = 0;
					
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

	delete circleFromTriangleCalculator;

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

				circleCandidates.at(i) = ChooseCircleWithMorePoints(circle, circleCandidates.at(i));
				matchFound = true;
			}
		}

		if (!matchFound)
			circleCandidates.push_back(circle);
	}
}

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

SimpleCircle SignCircleFinder::ChooseCircleWithMorePoints(SimpleCircle &sourceCircle, SimpleCircle &targetCircle)
{
	return sourceCircle.points > targetCircle.points ? sourceCircle : targetCircle;
}

	








