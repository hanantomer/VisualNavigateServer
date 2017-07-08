#include "global.h"
#include "SignCircleFinder.h"

#pragma once
class RoadSignsDetector
{

private:
	const int filterSquareSize = 10;
	const float minSignMatch = 0.5;

public:
	RoadSignsDetector();
	void Run(std::string imageFileName);
	~RoadSignsDetector();
};

