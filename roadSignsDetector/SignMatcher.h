#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;


#pragma once
class SignMatcher
{
private:
	vector<SignRef> _referenceSigns;
	int const _intensitySampleThreashold = 5;
	int const _intensityThreashold = 200;
	float const _maxDiscrepency = 10.0;
	float const _maxWeightedDiscrepency = 4.0;
	string sourcImageFileName;

	std::vector<std::vector<int>> GetIntensityChangeMatrix(cv::Mat &mat);
	float GetSimilarityScore(std::vector<std::vector<int>>, std::vector<std::vector<int>>);
	vector<int> GetRowDiscrepencies(vector<int> sourceLine, vector<int> targetLine);
	float GetTotalScore(float rowScoreCoeficient, vector<int> allGaps, vector<vector<int>>);

public:
	SignMatcher(string sourceFileName);
	~SignMatcher();
	void LoadReferenceSigns(std::string);
	SignRef GetMatchedSign(cv::Mat&, int);
};


