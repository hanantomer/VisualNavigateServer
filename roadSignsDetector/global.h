#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <Windows.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#define _USE_MATH_DEFINES

using namespace std;
using namespace cv;

#pragma once
struct SimplePoint
{
	int x;
	int y;
	double avgDistance;
	double slopeToNearestPoint;

	int GetDistance(SimplePoint other)
	{
		return (int)(sqrt(pow(x - other.x, 2) + pow(y - other.y, 2)));
	}
};

#pragma once
struct SimpleRow
{
	int y;
	std::vector<int> xVector;
};

#pragma once
struct SimpleCircle
{
	SimplePoint center;
	float radius;
	int points;
};

#pragma once
struct SignRef
{
	std::string fileName;
	std::vector<std::vector<int>> intensityChangeMatrix;
	cv::Mat mat;
	cv::Mat matStereo;
	float score;
};

const int maxCircleRadius = 75;
const int minCircleRadius = 40;


#pragma once
class utils
{
public:

	static void DisplayMat(Mat &mat, string windowMame)
	{
		cv::namedWindow(windowMame, WINDOW_AUTOSIZE);
		cv::imshow(windowMame, mat);
	}


	static void SaveFile(string &fileName, Mat &mat)
	{
		remove(fileName.c_str());
		imwrite(fileName, mat);
	}

	static Mat GetMaskCombinedBlack(Mat &src)
	{
		Mat hsv;
		cvtColor(src, hsv, CV_BGR2HSV);

		Scalar fromScalarRed = Scalar(140, 25, 35);
		Scalar toScalarRed = Scalar(180, 255, 255);

		Scalar fromScalarBlack = Scalar(0, 0, 0);
		Scalar toScalarBlack = Scalar(180, 255, 90);

		Mat maskRed;
		cv::inRange(hsv, fromScalarRed, toScalarRed, maskRed);

		return maskRed;
	}

	static Mat GetMaskCombinedWhite(Mat &src)
	{
		Mat hsv;
		cvtColor(src, hsv, CV_BGR2HSV);

		Scalar fromScalarWhite = Scalar(0, 0, 160);
		Scalar toScalarWhite = Scalar(180, 80, 255);
		Mat maskWhite;
		cv::inRange(hsv, fromScalarWhite, toScalarWhite, maskWhite);


		return maskWhite;
	}

	static void GetEdgePoints(Mat &src, Mat &edgePoints, Mat &edgePointsExtended, Mat &maskCombinedBlack)
	{

		maskCombinedBlack =
			GetMaskCombinedBlack(src);

		//cv::namedWindow("maskCombinedBlack", WINDOW_AUTOSIZE);
		//cv::imshow("maskCombinedBlack", maskCombinedBlack);

		Mat maskCombinedWhite =
			GetMaskCombinedWhite(src);

		//cv::namedWindow("maskWhite", WINDOW_AUTOSIZE);
		//cv::imshow("maskWhite", maskWhite);

		Mat result(maskCombinedWhite.rows, maskCombinedWhite.cols, CV_8UC1, Scalar(0, 0, 0));

		MarkEdgePoints(maskCombinedWhite, maskCombinedBlack, result, 255, 255);

		edgePoints = result;

		Mat resultCopy = result.clone();

		edgePointsExtended = result.clone();
		
		MarkEdgePoints(maskCombinedWhite, resultCopy, edgePointsExtended, 255, 255);
	}

	static void MarkEdgePoints(Mat &sourceMask, Mat &targetMask, Mat &result, uchar sourceColor, uchar targetColor)
	{
		int count = 0;
		for (int y = 0; y < sourceMask.rows; ++y) {
			for (int x = 0; x < sourceMask.cols; ++x) {
				uchar u = sourceMask.at<uchar>(y, x);
				if (sourceMask.at<uchar>(y, x) == sourceColor)
				{

					if (
						(x + 1 < targetMask.cols && targetMask.at<uchar>(y, x + 1) == targetColor)									|| // y+1
						(x > 0 && targetMask.at<uchar>(y, x - 1) == targetColor)													|| // y-1
						(x + 1 < targetMask.cols && y + 1 < targetMask.rows && targetMask.at<uchar>(y + 1, x + 1) == targetColor)	|| // y+1, x+1
						(x > 0 && y > 0 && targetMask.at<uchar>(y - 1, x - 1) == targetColor)										|| // y-1, x-1
						(y + 1 < targetMask.rows &&  targetMask.at<uchar>(y + 1, x) == targetColor)									|| // x+1
						(y > 0 && targetMask.at<uchar>(y - 1, x) == targetColor)													|| // x-1
						(y + 1 < targetMask.rows &&  x > 0 && targetMask.at<uchar>(y + 1, x - 1) == targetColor)					|| // x+1, y-1
						(y > 0 && x + 1 < targetMask.cols && targetMask.at<uchar>(y - 1, x + 1) == targetColor))					   // x-1, y+1

					{
						result.at<uchar>(y, x) = 255;
						count++;
					}
				}
			}
		}
	}

	/*
	calibrate circle to adjust to the sign red/blue frame
	*/
	static SimpleCircle CalibrateCircle(SimpleCircle &circle, Mat &maskCombinedBlack, Mat &source, float signFrameRelativeWidth)
	{
		Mat extractedSign =
			utils::ExtractCircle(source, cv::Point(circle.center.x, circle.center.y), circle.radius);

		SimpleCircle calibratedCircle = circle;

		int numberOfOutPoints = 0;

		float numberOfAngles = 8;

		double radianTotal = 2 * 3.14159265358979323846;

		cv::Point perimeterPoint;

		for (float i = 0; i < numberOfAngles; i++)
		{
			double theta = i / numberOfAngles * radianTotal;

			perimeterPoint.x = circle.center.x + (circle.radius * sin(theta));
			perimeterPoint.y = circle.center.y - (circle.radius * cos(theta));

			bool hasOutPoint = CalibratePoint(maskCombinedBlack, perimeterPoint, circle, signFrameRelativeWidth);

			numberOfOutPoints += (int)hasOutPoint;
		}

		if (numberOfOutPoints >= numberOfAngles / 2)
		{
			calibratedCircle.radius *= (signFrameRelativeWidth + 1);
		}

		return calibratedCircle;
	}


	static bool CalibratePoint(Mat &maskCombinedBlack, cv::Point perimeterPoint, SimpleCircle circle, float signFrameRelativeWidth)
	{
		cv::Point outPoint(circle.center.x, circle.center.y);

		float outPointDeltaY =
			((perimeterPoint.y - circle.center.y) * circle.radius * (1 + signFrameRelativeWidth)) /
			circle.radius;

		outPoint.y += ceil(outPointDeltaY);

		float a = outPoint.y - circle.center.y;
		float c = (1 + signFrameRelativeWidth) * circle.radius;
		float f =  pow(c, 2) - pow(a, 2);
		float outPointDeltaX = sqrt(ceil(f));

		if (perimeterPoint.x < circle.center.x)
			outPointDeltaX *= -1;

		outPoint.x +=
			perimeterPoint.x != circle.center.x ?
			ceil(outPointDeltaX) :
			0;

		LineIterator lit = 
			cv::LineIterator(maskCombinedBlack, perimeterPoint, outPoint, 8);

		int start_red_point = 0;
		int end_red_point = 0;

		bool darkFound = false;
		int darkCount = 0;

		// scan inside to outside
		for (int i = 0; i < lit.count; i++, ++lit)
		{
			cv::Point pt = lit.pos();
			
			uchar val = maskCombinedBlack.at<uchar>(pt);
			
			darkFound = val != '\0';

			if (darkFound)
			{
				darkCount++;
			}
			else
			{
				darkCount = 0;
			}

			if (darkCount >= 10)
				return true;

		}

		return false;
	}


	static int GetCirclePoints(SimpleCircle circle, Mat bwMat)
	{
		int numOfPoints = 0;


		Point circle_center(circle.center.x, circle.center.y);
		Size axes(circle.radius, circle.radius); 
		vector<Point> points_on_circle;
		ellipse2Poly(circle_center, axes, 0, 0, 360, 1, points_on_circle);

		for (int i = 0; i < points_on_circle.size(); i++) {
			Point current_point = points_on_circle[i];

			if (current_point.x > 0 && current_point.y > 0 &&
				current_point.x < bwMat.cols && current_point.y < bwMat.rows)
			{
				if (bwMat.at<unsigned char>(current_point) > 0) {
					numOfPoints++;
				}
			}
		}

		return numOfPoints;
	}

	static void FilterByDensity(Mat &mat, int filterSquareSize)
	{
		// loop square by square
		for (int siCols = 0; siCols*filterSquareSize < mat.cols; siCols++)
		{
			for (int siRows = 0; siRows*filterSquareSize < mat.rows; siRows++)
			{
				int colStart = siCols*filterSquareSize;

				int rowStart = siRows*filterSquareSize;

				vector<SimplePoint> whitePoints = utils::GetSquarePoints(mat, colStart, rowStart, filterSquareSize, filterSquareSize);

				if (whitePoints.size() == 0)
				{
					continue;
				}


				if (whitePoints.size() < 3 || whitePoints.size() > 25)
				{
					utils::ResetPoints(whitePoints, mat);
					continue;
				}


				double avgDistanceAll = utils::CalcualteAvgDistance(whitePoints);

				if (avgDistanceAll > 1.4)
				{
					utils::ResetPoints(whitePoints, mat);
					continue;
				}
			}
		}
	}

	static std::map<int, bool*> GetMatRows(cv::Mat mat)
	{
		std::map<int, bool*> rows;

		for (int y = 0; y < mat.rows; ++y) {

			bool *xArr = new bool[mat.cols];

			for (int x = 0; x < mat.cols; ++x) {

				uchar v = mat.at<uchar>(y, x);
				
				if (v == 255)
				{
					xArr[x] = true;
				}
				else
				{
					xArr[x] = false;
				}
			}

			rows[y] = xArr;
		}

		return rows;
	}

	static std::vector<SimplePoint> GetSquarePoints(cv::Mat mat, int xStart, int yStart, int squareSizeHor, int squareSizeVer)
	{
		std::vector<SimplePoint> points;

		for (int y = yStart; y < yStart + squareSizeVer && y < mat.rows; ++y) {
			for (int x = xStart; x < xStart + squareSizeHor && x < mat.cols; ++x) {

				assert(x < mat.cols);
				assert(y < mat.rows);


				uchar v = mat.at<uchar>(y, x);
				if (v == 255)
				{
					SimplePoint sp;
					sp.x = x;
					sp.y = y;
					points.push_back(sp);
				}
			}
		}

		return points;
	}

	static std::vector<SimplePoint> FilterConsecutivePoints(std::vector<SimplePoint> points)
	{
		return points;


		if (points.size() < 10)
			return points;

		utils::bubbleSortY(points);

		std::vector<SimplePoint> pointsFilteredY;

		pointsFilteredY.push_back(points.at(0));

		for (size_t i = 1; i < points.size(); i++)
		{
			if (points.at(i).y != points.at(i - 1).y || abs(points.at(i).x - points.at(i - 1).x) > 6)
			{
				pointsFilteredY.push_back(points.at(i));
			}
		}

		utils::bubbleSortX(pointsFilteredY);

		std::vector<SimplePoint> pointsFilteredX;

		pointsFilteredX.push_back(pointsFilteredY.at(0));

		for (size_t i = 1; i < pointsFilteredY.size(); i++)
		{
			if (pointsFilteredY.at(i).x != pointsFilteredY.at(i - 1).x || abs(points.at(i).y - points.at(i - 1).y) > 6)
			{
				pointsFilteredX.push_back(pointsFilteredY.at(i));
			}
		}

		return pointsFilteredX;
	}

	static int GetDistance(SimplePoint &p1, SimplePoint &p2)
	{
		return (int)sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	}

	static void ResetPoints(vector<SimplePoint> points, cv::Mat &mat)
	{
		for (size_t i = 0; i < points.size(); i++)
		{

			assert(points.at(i).x < mat.cols);
			assert(points.at(i).y < mat.rows);


			uchar v = mat.at<uchar>(points.at(i).y, points.at(i).x);
			v = 0;
			mat.at<uchar>(points.at(i).y, points.at(i).x) = v;
		}
	}

	static double CalcualteAvgDistance(std::vector<SimplePoint> whitePoints)
	{
		double sumDistance = 0;

		for (size_t i = 1; i < whitePoints.size(); i++)
		{
			
			int nearestPointDistance =
				max(
					abs(whitePoints.at(i).x - whitePoints.at(i - 1).x),
					abs(whitePoints.at(i).y - whitePoints.at(i - 1).y));

			sumDistance += nearestPointDistance;

		}


		double avg = sumDistance /= whitePoints.size();

		return avg;
	}

	static cv::Mat3b ExtractCircle(cv::Mat img, cv::Point center, int radius)
	{
		cv::Mat1b mask(img.size(), uchar(0));

		circle(mask, center, radius, cv::Scalar(255), CV_FILLED);

		// Create a black image
		cv::Mat3b res(img.size(), cv::Vec3b(255, 255, 255));

		// Copy only the image under the white circle to black image
		img.copyTo(res, mask);

		// Compute the bounding box
		cv::Rect bbox(center.x - radius, center.y - radius, 2 * radius, 2 * radius);

		// Crop according to the roi
		res = res(bbox);

		return res;
	}

	static void bubbleSort(std::vector<SimplePoint> &vector) {
		size_t n = vector.size();
		bool exchanges;
		do {
			exchanges = false;  // assume no exchanges
			for (int i = 0; i < n - 1; i++) {
				if (vector.at(i).avgDistance < vector.at(i + 1).avgDistance) {
					SimplePoint temp = vector.at(i);
					vector.at(i) = vector.at(i + 1);
					vector.at(i + 1) = temp;
					exchanges = true;  // after exchange, must look again
				}
			}
		} while (exchanges);
	}

	static void bubbleSortX(std::vector<SimplePoint> &vector) {

		if (vector.size() < 2)
			return;

		size_t n = vector.size();
		bool exchanges;
		do {
			exchanges = false;  // assume no exchanges
			for (int i = 0; i < n - 1; i++) {
				if (vector.at(i).x < vector.at(i + 1).x) {
					SimplePoint temp = vector.at(i);
					vector.at(i) = vector.at(i + 1);
					vector.at(i + 1) = temp;
					exchanges = true;  // after exchange, must look again
				}
			}
		} while (exchanges);
	}

	static void bubbleSortY(std::vector<SimplePoint> &vector) {

		if (vector.size() < 2)
			return;

		size_t n = vector.size();
		bool exchanges;
		do {
			exchanges = false;  // assume no exchanges
			for (int i = 0; i < n - 1; i++) {
				if (vector.at(i).y < vector.at(i + 1).y) {
					SimplePoint temp = vector.at(i);
					vector.at(i) = vector.at(i + 1);
					vector.at(i + 1) = temp;
					exchanges = true;  // after exchange, must look again
				}
			}
		} while (exchanges);
	}


	static void bubbleSort(std::vector<SignRef> &vector) {
		size_t n = vector.size();
		bool exchanges;
		do {
			exchanges = false;  // assume no exchanges
			for (int i = 0; i < n - 1; i++) {
				if (vector.at(i).score < vector.at(i + 1).score) {
					SignRef temp = vector.at(i);
					vector.at(i) = vector.at(i + 1);
					vector.at(i + 1) = temp;
					exchanges = true;  // after exchange, must look again
				}
			}
		} while (exchanges);
	}

	static void bubbleSort(int x[], int n) {
		bool exchanges;
		do {
			exchanges = false;  // assume no exchanges
			for (int i = 0; i < n - 1; i++) {
				if (x[i] < x[i + 1]) {
					int temp = x[i]; x[i] = x[i + 1]; x[i + 1] = temp;
					exchanges = true;  // after exchange, must look again
				}
			}
		} while (exchanges);
	}

	static void bubbleSort(float x[], int n) {
		bool exchanges;
		do {
			exchanges = false;  // assume no exchanges
			for (int i = 0; i < n - 1; i++) {
				if (x[i] < x[i + 1]) {
					float temp = x[i]; x[i] = x[i + 1]; x[i + 1] = temp;
					exchanges = true;  // after exchange, must look again
				}
			}
		} while (exchanges);
	}

	static cv::Mat ConvertToSobel(Mat image)
	{

		GaussianBlur(image, image, Size(3, 3), 0, 0, BORDER_DEFAULT);

		Mat grey;

		cvtColor(image, grey, CV_BGR2GRAY);

		Mat grad_x, grad_y;
		Mat abs_grad_x, abs_grad_y;

		Mat sobelx;
		Sobel(grey, sobelx, CV_32F, 1, 0);

		double minVal, maxVal;
		minMaxLoc(sobelx, &minVal, &maxVal); //find minimum and maximum intensities

		Mat sobel;
		sobelx.convertTo(sobel, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

		return sobel;
	}

	static bool CircleContains(SimpleCircle &circle1, SimpleCircle &circle2)
	{
		if (circle1.center.x + circle1.radius > circle2.center.x + circle1.radius &&
			circle1.center.x - circle1.radius < circle2.center.x - circle1.radius &&
			circle1.center.y + circle1.radius > circle2.center.y + circle1.radius &&
			circle1.center.y - circle1.radius < circle2.center.y - circle1.radius)
			return true;

		return false;
	}
};



