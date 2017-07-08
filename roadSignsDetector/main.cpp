#include "RoadSignsDetector.h"


int main(int argc, char** argv)
{
	//TODO validate argument

	string imageFileName(argv[1]);

	RoadSignsDetector *rsd = new RoadSignsDetector();

	rsd->Run(imageFileName);
};

