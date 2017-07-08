#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include "LogWrapper.h"
#include "SnapshotAnalyzer.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;
namespace fs = boost::filesystem;
//using namespace cv::xfeatures2d;

cv::Mat _image;
Mat imageResized;

std::string _fileName;
int _winH = 800;
int _winW = 1200;
int _scrolHight = 0;
int _scrolWidth = 0;

LogWrapper* _log;

SnapshotAnalyzer::SnapshotAnalyzer(const std::string fileName)
{
	_fileName = fileName;

	_log = new LogWrapper();
}

void SnapshotAnalyzer::CrossMatch()
{
	// Default parameters of ORB
	int nfeatures = 75;
	float scaleFactor = 1.2f;
	int nlevels = 5;
	int edgeThreshold = 31; // Changed default (31);
	int firstLevel = 0;
	int WTA_K = 2;
	int scoreType = ORB::HARRIS_SCORE;
	int patchSize = 31;
	int fastThreshold = 20;

	const double nn_match_ratio = 0.25f; // Nearest-neighbour matching ratio
	
	Ptr<ORB> detector = ORB::create(
		nfeatures,
		scaleFactor,
		nlevels,
		edgeThreshold,
		firstLevel,
		WTA_K,
		scoreType,
		patchSize,
		fastThreshold);


	//fs::path targetDir("c:\\Dev\\roadview\\Photos\\normalized");
	fs::path targetDir("c:\\Dev\\roadview\\Photos");

	fs::directory_iterator it1(targetDir), eod1;

	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

	std::vector<KeyPoint> kp1;

	cv::Mat sourceDescriptors;

	Mat img_1;

	std::string file1Name;
	std::string file2Name;

	BOOST_FOREACH(fs::path const &p1, std::make_pair(it1, eod1))
	{
		if (fs::is_regular_file(p1))
		{
			vector< vector<DMatch> > matches;

			Mat img_2;

			file1Name = p1.string();

			img_1 = imread(file1Name, IMREAD_GRAYSCALE);

			std::vector<KeyPoint> kp2;

			cv::Mat targetDescriptors;

			size_t maxNumMatches = 0;

			std::string matcdedFileName;

			detector->detectAndCompute(img_1, noArray(), kp1, sourceDescriptors);

			fs::directory_iterator it2(targetDir), eod2;

			BOOST_FOREACH(fs::path const &p2, std::make_pair(it2, eod2))
			{
				file2Name = p2.string();

				if (file1Name != file2Name)
				{
					img_2 = imread(file2Name, IMREAD_GRAYSCALE);

					if (fs::is_regular_file(p2))
					{
						detector->detectAndCompute(img_2, cv::Mat(), kp2, targetDescriptors);

						matches.clear();

						matcher->knnMatch(sourceDescriptors, targetDescriptors, matches, 2);

						/*
						Mat res;
						drawMatches(img_1, kp1, img_2, kp2, matches, res,
							Scalar(255, 0, 0), Scalar(255, 0, 0));

						imshow("Display window", img_1);
						*/

						//break;

						int matchIdx = 0;

						for (unsigned i = 0; i < matches.size(); i++) {

							//cout << matches[i][0].distance << "-" << matches[i][1].distance << "\r\n";
							if (abs( matches[i][0].distance - matches[i][1].distance) < 2) {
								matchIdx++;
							}
						}

						


						cout 
							<< file1Name.substr(file1Name.find_last_of('\\') + 1)
							<< "-" 
							<< file2Name.substr(file2Name.find_last_of('\\') + 1) 
							<< ","
							<< matchIdx
							<< "- matches:"
							<< "\r\n";

						if (matchIdx > maxNumMatches)
						{
							maxNumMatches = matchIdx;
							matcdedFileName = file2Name;
						}
					}
				}
			}

			//break;

			cout 
				<< "best match:" 
				<< file1Name.substr(file1Name.find_last_of('\\') + 1) 
				<< "-"  
				<< matcdedFileName.substr(matcdedFileName.find_last_of('\\') + 1)
				<< "\r\n";
		}
	}
}


void SnapshotAnalyzer::Normalize()
{
	_image = imread(_fileName, IMREAD_UNCHANGED); // Read the file

	if (!_image.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return;
	}

	cv::Size size;
	cv::resize(_image, imageResized, size, 0.5, 0.5);
	_image = imageResized;
	
	this->ConvertToSobel();

	size_t extPos = _fileName.find_last_of('.');

	std::string convertedFileName =
		_fileName.substr(0, extPos) + "_1" + _fileName.substr(extPos, _fileName.size() - extPos - 3) + "bmp";

	imwrite(convertedFileName, _image);
}

void SnapshotAnalyzer::ShowImage()
{
	Mat winImage = _image(Rect(_scrolWidth, _scrolHight, _winW, _winH));
	imshow("Display window", winImage);
}

void SnapshotAnalyzer::CutMiddle()
{
	cv::Rect leftROI(0, 0, 100, 1000);
	cv::Mat left = _image(leftROI);

	cv::Rect rightROI(650, 0, 550, 1000);
	cv::Mat right = _image(rightROI);

	cv::hconcat(left, right, _image);
}


void SnapshotAnalyzer::CannyThreshold()
{
	int lowThreshold = 0;
	int const max_lowThreshold = 100;

	int ratio = 3;

	Mat src_gray;
	Mat dst, detected_edges;

	int kernel_size = 3;

	dst.create(_image.size(), _image.type());

	/// Convert the image to grayscale
	cvtColor(_image, src_gray, CV_BGR2GRAY);

	/// Reduce noise with a kernel 3x3
	blur(src_gray, detected_edges, Size(3, 3));

	/// Canny detector
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);

	_image.copyTo(dst, detected_edges);

	_image = dst;
}


void SnapshotAnalyzer::ConvertToSobel()
{

	GaussianBlur(_image, _image, Size(3, 3), 0, 0, BORDER_DEFAULT);

	Mat grey;

	cvtColor(_image, grey, CV_BGR2GRAY);

	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	Mat sobelx;
	Sobel(grey, sobelx, CV_32F, 1, 0);

	double minVal, maxVal;
	minMaxLoc(sobelx, &minVal, &maxVal); //find minimum and maximum intensities

	Mat draw;
	sobelx.convertTo(draw, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

	_image = draw;
}

//Mat SnapshotAnalyzer::SubstractBackground(Mat image)
//{
//	Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
//}

 
SnapshotAnalyzer::~SnapshotAnalyzer()
{
}
