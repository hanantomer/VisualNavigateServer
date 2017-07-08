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



void RoadSignsDetector::Run(string imageFileName)
{
	SignMatcher *signMatcher = new SignMatcher();

	// source
	Mat  src = imread(imageFileName);
	
	//cv::resize(src, src, cv::Size(), 0.5, 0.5);

	//cv::namedWindow("src", WINDOW_AUTOSIZE);
	//cv::imshow("src", src);

	Mat maskCombinedBlack;
	Mat whiteCloseToBlack;
	Mat whiteCloseToBlackExtended;
	utils::GetEdgePoints(src, whiteCloseToBlack, whiteCloseToBlackExtended, maskCombinedBlack);
	

	cv::namedWindow("w1", WINDOW_AUTOSIZE);
	cv::imshow("w1", whiteCloseToBlack);

	Mat whiteCloseToBlackFiltered = whiteCloseToBlack.clone();

	utils::FilterByDensity(whiteCloseToBlackFiltered, filterSquareSize);

	cv::namedWindow("w2", WINDOW_AUTOSIZE);
	cv::imshow("w2", whiteCloseToBlackFiltered);

	
	SignCircleFinder *signCircleFinder = new SignCircleFinder();
	vector<SimpleCircle> signCircles = signCircleFinder->FindCircleSigns(whiteCloseToBlackFiltered, whiteCloseToBlack, whiteCloseToBlackExtended, maskCombinedBlack, src);

	Mat srcForCircleExtraction = src.clone();

	Mat3b res(src.rows, src.cols, Vec3b(0, 0, 0));

	src.copyTo(res(Rect(0, 0, src.cols, src.rows)));

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

			SignRef matchedSign = signMatcher->GetMatchedSign(circ, i);



			//remove(circleFileName.c_str());
			//imwrite(circleFileName, circ);

			if (matchedSign.score >= minSignMatch)
			{
				matchedSign.mat.copyTo(res(Rect(0 + 100 * i, 0, matchedSign.mat.cols, matchedSign.mat.rows)));
			}
		}
	}

	//cv::resize(src, src, cv::Size(), 0.25, 0.25);

	cv::namedWindow("srcCircles", WINDOW_AUTOSIZE);
	cv::imshow("srcCircles", src);

	cv::resize(res, res, cv::Size(), 0.5, 0.5);

	cv::namedWindow("final", WINDOW_AUTOSIZE);
	cv::imshow("final", res);


	// write
	string convertedFileName = imageFileName + "_signs.bmp";

	remove(convertedFileName.c_str());
	imwrite(convertedFileName, src);
	

	waitKey(0);

}




RoadSignsDetector::~RoadSignsDetector()
{
}
