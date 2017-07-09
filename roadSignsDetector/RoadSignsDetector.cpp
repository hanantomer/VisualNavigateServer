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


RoadSignsDetector::RoadSignsDetector()
{
}

vector<SimpleCircle> RoadSignsDetector::FindCircles(Mat &src)
{
	Mat maskCombinedBlack;
	Mat whiteCloseToBlack;
	Mat whiteCloseToBlackExtended;

	utils::GetEdgePoints(src, whiteCloseToBlack, whiteCloseToBlackExtended, maskCombinedBlack);
//	utils::DisplayMat(whiteCloseToBlack, "whiteCloseToBlack");

	Mat whiteCloseToBlackFiltered = whiteCloseToBlack.clone();
	utils::FilterByDensity(whiteCloseToBlackFiltered, filterSquareSize);
//	utils::DisplayMat(whiteCloseToBlackFiltered, "whiteCloseToBlackFiltered");

	SignCircleFinder *signCircleFinder = new SignCircleFinder();
	vector<SimpleCircle> signCircles = signCircleFinder->FindCircleSigns(whiteCloseToBlackFiltered, whiteCloseToBlack, whiteCloseToBlackExtended, maskCombinedBlack, src);

	return signCircles;
}

Mat3b RoadSignsDetector::CreateResultMatrix(Mat &src)
{
	Mat3b res(src.rows, src.cols, Vec3b(0, 0, 0));

	src.copyTo(res(Rect(0, 0, src.cols, src.rows)));

	return res;
}

void RoadSignsDetector::DisplaySigns(Mat &src, Mat &res, vector<SimpleCircle> &signCircles, string &imageFileName)
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


		if (i < 10)
		{

			Mat circ =
				utils::ExtractCircle(srcForCircleExtraction, cv::Point(signCircles.at(i).center.x, signCircles.at(i).center.y), signCircles.at(i).radius);

			// write

			string circleFileName = imageFileName + "_cicle" + to_string(i) + ".bmp";

			SignMatcher *signMatcher = new SignMatcher();

			SignRef matchedSign = signMatcher->GetMatchedSign(circ, i);

			if (matchedSign.score >= minSignMatch)
			{
				matchedSign.mat.copyTo(res(Rect(0 + 100 * i, 0, matchedSign.mat.cols, matchedSign.mat.rows)));
			}
		}
	}

	//utils::DisplayMat(src, "srcCircles");


	cv::resize(res, res, cv::Size(), 0.5, 0.5);
	utils::DisplayMat(res, "final");


	// write
	string convertedFileName = imageFileName + "_signs.bmp";
	utils::SaveFile(convertedFileName, res);
}

void RoadSignsDetector::Run(string imageFileName)
{
	Mat  src = imread(imageFileName);

	vector<SimpleCircle> signCircles =	this->FindCircles(src);

	Mat3b res = this->CreateResultMatrix(src);

	this->DisplaySigns(src, res, signCircles, imageFileName);

	waitKey(0);
}




RoadSignsDetector::~RoadSignsDetector()
{
}
