#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/video/video.hpp>
//#include <iostream>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//
#include "working_consts.h"
//

const int		Nreads = 10;

class Tracker {
private:
    vector<Point2f> trackedFeatures; 
    Mat             prevGrayROI;


public:
	Rect			current_trackingROI;
	/* original target properties */
	bool			newTargetSituation = false;
	Rect			OriginalTargetROI;
	Mat				OriginalTarget;
	Mat				grayOriginalTarget;
	vector<Point2f> OriginalTargetFeatures; 	
	Rect			TrackerROI; 
	double			TrkErrX_Avg=0;  // average for 3 readings	
	double			TrkErrX_Readings[Nreads];
	int				TrkErrX_readIndex = 0;


	int				TrackPercent = 0;		// % of the feature points that has correspondance to the next/previous frame
    bool            freshStart = true;
    Mat_<float>     rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix;
	int				min_features				= 5;// depend on light conditions 10;//40//200; 
	int				num_of_maxCornersFeatures	= 300;//300;
	float			mid_level_percent			= 1.25;		//	1/1.25=80%

	int		alphaSlider       = 0;
	int		alphaSlider2      = 0;
	char	TrackbarName[50]  = "StatusSum";
	char	TrackbarName2[50] = "StatusSumPercent";

	int		alphaSlider_max=100;

    ///Tracker();

	void setNewTarget(
		Rect			OriginalTargetROI,
		Mat				OriginalTarget,
		Rect			TrackerROI);

	// both imgTarget, imgROI should be of the same size.
    void processImage(Mat imgTarget,  SYSTEM_STATUS external_state);

};

#endif  // FEATURETRACKER_H