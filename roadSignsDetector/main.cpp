#include "RoadSignsDetector.h"


int main(int argc, char** argv)
{
	//TODO validate argument
	RoadSignsDetector *rsd = new RoadSignsDetector(string(argv[1]));

	rsd->Run();
};

