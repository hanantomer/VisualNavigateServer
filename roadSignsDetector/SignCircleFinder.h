#include "global.h"
#pragma once

class SignCircleFinder
{
public:
	SignCircleFinder(Mat source, string sourceImageFileName);
	vector<SimpleCircle> Run();
	virtual ~SignCircleFinder();
private:
	const bool doFilter = false;
	const bool doCalibrate = true;
	const bool doMatchNearCircles = true;
	const int filterSquareSize = 10;
	const int circle_match_threshold = 6;
	const int min_frequency_threshold = 10;
	const int position_calibration_threshold = 15;
	const int radius_calibration_threshold = 1;
	const int min_cicle_points = 50;
	const float SIGN_FRAME_RELATIVE_WIDTH = 0.33;
	int usedCount = 0;
	string sourceImageFileName;

	Mat source;
	Mat maskCombinedBlack;
	Mat whiteCloseToBlack;
	Mat whiteCloseToBlackExtended;
	Mat whiteCloseToBlackFiltered;
	
	map<int, bool*> whiteRows;
	map<int, bool*> whiteRowsFiltered; 

	vector<SimpleCircle> FindCircles();
	void CreateExtendedMatrixes();
	bool CircleIsMatched(SimpleCircle circle, SimpleCircle simpleCircle);
	SimpleCircle ChooseCircleWithMorePoints(SimpleCircle &sourceCircle, SimpleCircle &targetCircle);
	void StoreCircle(SimpleCircle circle, vector<SimpleCircle> &simpleCircleVector);
	SimpleCircle GetRowCircle(int  y, bool* xArrFiltered, map<int, bool*>&, Mat whiteCloseToBlack, Mat whiteCloseToBlackExtended, map<int, bool*>&);
	int GetTupleNextX(int nextXStart, int nextXEnd, bool* xArr[10]);
	vector<SimplePoint> GetTupleNextYPoints(int nextXStart, int nextXEnd, int minY, int maxY,  map<int, bool*>& whiteRows);
	void ConsolidateCircle(SimpleCircle&, SimpleCircle&);
	SimpleCircle CalibrateCircle(SimpleCircle circle, Mat &maskCombinedBlack, Mat &source);
};
