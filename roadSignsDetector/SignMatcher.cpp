#include "global.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "opencv2/features2d.hpp"
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include "SignMatcher.h"

namespace fs = boost::filesystem;

SignMatcher::SignMatcher()
{
	LoadReferenceSigns("C:\\RoadSigns\\Israel\\Circles");
}

std::vector<std::vector<int>> SignMatcher::GetIntensityChangeMatrix(cv::Mat &mat)
{
	std::vector<std::vector<int>> intensityChangeMatrix;

	uchar last = 0;

	int matrixRowIndex = 0;

	int edgeIgnoreCount = 5;
	
	for (int y = edgeIgnoreCount; y < mat.rows - edgeIgnoreCount; ++y) {

		intensityChangeMatrix.push_back(vector<int>());

		int lastXChange = 0;

		for (int x = edgeIgnoreCount; x < mat.cols - 5; x++) {

			uchar current = mat.at<uchar>(y, x);

			if (x > edgeIgnoreCount && last != current && x -lastXChange > 1)
			{
				intensityChangeMatrix.at(matrixRowIndex).push_back(x);
				mat.at<uchar>(y, x) = 255;
				lastXChange = x;
			}
			else
			{
				mat.at<uchar>(y, x) = 0;
			}

			last = current;
		}

		matrixRowIndex++;
	}


	return intensityChangeMatrix;
}

void SignMatcher::LoadReferenceSigns(std::string directory)
{
	fs::path targetDir(directory);

	fs::directory_iterator it1(targetDir), eod1;

	BOOST_FOREACH(fs::path const &p1, std::make_pair(it1, eod1))
	{
		if (fs::is_regular_file(p1))
		{
			try {

				std::vector<KeyPoint> targetKeypoints;

				SignRef sr;

				sr.fileName = p1.string();

				Mat signMat = cv::imread(sr.fileName);

				Mat signMatStereo = utils::GetMaskCombinedWhite(signMat);

				//cv::namedWindow("signMatStereo" + sr.fileName, WINDOW_AUTOSIZE);
				//cv::imshow("signMatStereo" + sr.fileName, signMatStereo);


				sr.intensityChangeMatrix = GetIntensityChangeMatrix(signMatStereo);
				sr.matStereo = signMatStereo;
				sr.mat = signMat;
				_referenceSigns.push_back(sr);

				// write
				//string signFileName = sr.fileName.substr(0, sr.fileName.find('.')) + "_stereo.png";
				//remove(signFileName.c_str());
				//imwrite(signFileName, signMatStereo);

			}
			catch (Exception e)
			{
				cerr << e.err;
			}
		}
	}
}

SignRef SignMatcher::GetMatchedSign(Mat &sourceMat, int si)
{
	static int sourceIndex = 0;
	sourceIndex++;

	std::string matcdedFileName;

	//sourceMat = utils::GetMaskCombinedWhite(sourceMat);
	//cv::namedWindow("source" + to_string(si), WINDOW_AUTOSIZE);
	//cv::imshow("source" + to_string(si), source);

	string matchedFileName = "";
	float maxScore = 0;

	vector<SignRef> matScoreVector;

	cv::Size targetSize = _referenceSigns.at(0).matStereo.size();

	cv::resize(sourceMat, sourceMat, targetSize);


	Mat signMatStereo = utils::GetMaskCombinedWhite(sourceMat);

	// write
	string signFileName = "source" + to_string(sourceIndex) + "_stereo.png";
	remove(signFileName.c_str());
	imwrite(signFileName, signMatStereo);



	std::vector<std::vector<int>> sourceIntensityChangeMatrix = GetIntensityChangeMatrix(signMatStereo);


	for (size_t refIndex = 0; refIndex < _referenceSigns.size(); refIndex++)
	{
		vector<vector<int>> refIntensityChangeMatrix = _referenceSigns.at(refIndex).intensityChangeMatrix;

		float similarityScore =
			GetSimilarityScore(sourceIntensityChangeMatrix, refIntensityChangeMatrix);

		SignRef signRef;
		signRef = _referenceSigns.at(refIndex);
		signRef.score = similarityScore;

		matScoreVector.push_back(signRef);
	}

	utils::bubbleSort(matScoreVector);
	
	return matScoreVector.at(0);
}

float SignMatcher::GetSimilarityScore(std::vector<std::vector<int>> sourceIntensityChangeMatrix, vector<vector<int>> refIntensityChangeMatrix)
{

	int minSize =
		sourceIntensityChangeMatrix.size() < refIntensityChangeMatrix.size() ?
		sourceIntensityChangeMatrix.size() :
		refIntensityChangeMatrix.size();

	float rowScoreCoeficient = 1.0 / minSize;

	int maxRowShift = 5;

	vector<vector<int>> allDiscrepencies;

	vector<int> allGaps; // diff between source size and target size

	for (int line = 0; line < minSize; line++)
	{
		int minRowPointsGap = 10;

		int selectedShift = 0;

		vector<int> rowDiscrepencies;

		for (int rowShift = 0; rowShift < maxRowShift; rowShift++)
		{
			if (line + rowShift >= minSize)
				continue;

			vector<int> rowShiftDiscrepencies;

			int rowPointsGap =
				(int)(abs((double)(refIntensityChangeMatrix.at(line + rowShift).size()) - (double)(sourceIntensityChangeMatrix.at(line).size())));
				

			if (rowPointsGap < minRowPointsGap)
			{
				minRowPointsGap = rowPointsGap;
				selectedShift = rowShift;
			}
		}

		vector<int> discrepencies =
			GetRowDiscrepencies(sourceIntensityChangeMatrix.at(line), refIntensityChangeMatrix.at(line + selectedShift));

		allDiscrepencies.push_back(discrepencies);

		allGaps.push_back(minRowPointsGap);
	}

	return GetTotalScore(rowScoreCoeficient, allGaps, allDiscrepencies);
}

float  SignMatcher::GetTotalScore(float rowScoreCoeficient, vector<int> allGaps, vector<vector<int>> allDiscrepencies)
{
	float totalScore = 0;

	int prevDiscrepencyCount = 3;

	for (size_t i = 0; i < allDiscrepencies.size(); i++)
	{
		float rowScore = 0;

		int rowSize = allDiscrepencies.at(i).size();

		for (size_t j = 0; j < rowSize; j++)
		{

			int discrepency = abs(allDiscrepencies.at(i).at(j));

			float score = 0;

			if (discrepency > _maxDiscrepency)
				continue;

			if (i > prevDiscrepencyCount)
			{
				float weightedDiscrepency = 0;

				for (size_t k = 1; k <= prevDiscrepencyCount; k++)
				{
					if (j < allDiscrepencies.at(i - k).size())
					{
						int prevDiscrepency = abs(allDiscrepencies.at(i - k).at(j));

						weightedDiscrepency += abs(prevDiscrepency - discrepency);
					}
					else
					{
						weightedDiscrepency += _maxWeightedDiscrepency;
					}
				}

				weightedDiscrepency /= prevDiscrepencyCount;

				//int prevDiscrepency = abs(allDiscrepencies.at(i -1).at(j));

				//int  weightedDiscrepency = abs(prevDiscrepency - discrepency);

				if (weightedDiscrepency > _maxWeightedDiscrepency)
					continue;

				score = ((_maxWeightedDiscrepency - weightedDiscrepency) / _maxWeightedDiscrepency);

			}
			else
			{
				score = (_maxDiscrepency - discrepency) / _maxDiscrepency;
			}

			assert(score >= 0 && score <= 1.0);

			score /= (float)(rowSize);

			rowScore += score;
		}

		assert(rowScore >= 0 && rowScore <= 1.0);

		rowScore *= rowScoreCoeficient;

		int gap = abs(allGaps.at(i));

		if (rowSize > 0 && rowScore > 0)
		{
			float gapPenalty = (float)rowSize / (((float)rowSize + (float)gap));
			if (gapPenalty < 1)
			{
				rowScore *= gapPenalty;
			}
		}

		totalScore += rowScore;
	}

	return totalScore;

}

vector<int> SignMatcher::GetRowDiscrepencies(vector<int> sourceLine, vector<int> targetLine)
{
	vector<int> discrepencies;

	for (int point = 0; point < sourceLine.size() && point < targetLine.size(); point++)
	{
		double discrepency = abs(sourceLine.at(point) - targetLine.at(point));

		discrepencies.push_back(discrepency);

	}
	
	return discrepencies;
}

SignMatcher::~SignMatcher()
{
}
