#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include "LogWrapper.h"
#include "opencv2/imgproc.hpp"
#include "SignCircleFinder.h"
#include "global.h"
#include "RoadSignsDetector.h"
#include "SignMatcher.h"


using namespace cv;
using namespace std;


RoadSignsDetector::RoadSignsDetector(std::string imageFilePath)
{
	this->sourceImage = imread(imageFilePath);

	int fileNameStartPos = imageFilePath.rfind('\\');

	this->sourceImageFileName = imageFilePath.substr(fileNameStartPos + 1);
}

void RoadSignsDetector::Run()
{
	vector<SimpleCircle> signCircles = this->FindCircles(this->sourceImage);

	Mat3b res = this->CreateResultMatrix(this->sourceImage);

	this->DisplaySigns(this->sourceImage, res, signCircles, sourceImageFileName);

	waitKey(0);
}



vector<SimpleCircle> RoadSignsDetector::FindCircles(Mat &source)
{
	SignCircleFinder *signCircleFinder = new SignCircleFinder(source, this->sourceImageFileName);

	vector<SimpleCircle> signCircles = signCircleFinder->Run();

	return signCircles;
}

Mat3b RoadSignsDetector::CreateResultMatrix(Mat &src)
{
	Mat3b res(src.rows, src.cols, Vec3b(0, 0, 0));

	src.copyTo(res(Rect(0, 0, src.cols, src.rows)));

	return res;
}

void RoadSignsDetector::DisplaySigns(Mat &src, Mat &res, vector<SimpleCircle> &signCircles, string &imageFilePath)
{

	Mat srcForCircleExtraction = src.clone();


	for (size_t i = 0; i < signCircles.size(); i++)
	{

		if (signCircles.at(i).center.x - signCircles.at(i).radius < 0)			// top
			continue;

		if (signCircles.at(i).center.x + signCircles.at(i).radius >  src.rows) // bottom
			continue;

		if (signCircles.at(i).center.y - signCircles.at(i).radius < 0)			// left
			continue;

		if (signCircles.at(i).center.y + signCircles.at(i).radius > src.cols)	// right
			continue;

		circle(src, cv::Point(signCircles.at(i).center.x, signCircles.at(i).center.y), signCircles.at(i).radius, Scalar(255, 255, 255));


		if (i < 50)
		{

			Mat circ =
				utils::ExtractCircle(srcForCircleExtraction, cv::Point(signCircles.at(i).center.x, signCircles.at(i).center.y), signCircles.at(i).radius);

			// write

			string circleFileName = imageFilePath + "_cicle" + to_string(i) + ".bmp";

			SignMatcher *signMatcher = new SignMatcher(imageFilePath);

			SignRef matchedSign = signMatcher->GetMatchedSign(circ, i);

			if (matchedSign.score >= minSignMatch)
			{
				matchedSign.mat.copyTo(res(Rect(0 + 100 * i, 0, matchedSign.mat.cols, matchedSign.mat.rows)));
			}
		}
	}

	utils::DisplayMat(src, "srcCircles");

	cv::resize(res, res, cv::Size(), 0.25, 0.25);
	utils::DisplayMat(res, "final");


	// write
	string signsFileName = "signs.bmp";

	utils::SaveFile(signsFileName, res, sourceImageFileName);
}

RoadSignsDetector::~RoadSignsDetector()
{
}
