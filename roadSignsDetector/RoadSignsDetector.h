#include "global.h"
#include "SignCircleFinder.h"

#pragma once
class RoadSignsDetector
{

private:
	const int filterSquareSize = 10;
	const float minSignMatch = 0.5;
	vector<SimpleCircle> FindCircles(Mat &src);
	Mat3b CreateResultMatrix(Mat &src);
	void DisplaySigns(Mat &src, Mat &res, vector<SimpleCircle> &signCircles, string &imageFileName);

public:
	RoadSignsDetector();
	void Run(std::string imageFileName);
	~RoadSignsDetector();
};

