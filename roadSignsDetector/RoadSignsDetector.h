#include "global.h"
#include "SignCircleFinder.h"

#pragma once
class RoadSignsDetector
{

private:
	const float minSignMatch = 0.3;
	string sourceImageFileName;
	Mat sourceImage;
	vector<SimpleCircle> FindCircles(Mat &src);
	Mat3b CreateResultMatrix(Mat &src);
	void DisplaySigns(Mat &src, Mat &res, vector<SimpleCircle> &signCircles, string &imageFileName);

public:
	RoadSignsDetector(std::string imageFilePath);
	void Run();
	~RoadSignsDetector();
};

