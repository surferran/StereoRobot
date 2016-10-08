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
//#include "working_consts.h"
#include "StereoRobotApp.hpp"
#include "Target.hpp"
//

const int		Nreads = 10;

class Tracker {
private:
    vector<Point2f> trackedFeatures,
					trackedFeaturesBack; 
    Mat             prevGrayROI;

	struct frameProperties{
		Mat				grayImage;
		vector<Point2f> goodFeaturesCoor; 
		Rect			relevantROI;	//  bounding the calculated feature points. 
		//-								//	using for next image searching area for features-flow.
	};
	frameProperties prevImProp,
					currentImProp;


	/* criteria variables(contants) */
	double	GFTFquality;
	double	GFTFminDistance;
	int		minFPsize_toLEARN;			// number of feature points
	int		minROIareaRatio_toLEARN;	// percent of ratio
	int		minFPsize_toTRACK;			// "
	int		minROIareaRatio_toTRACK;	// "
	int		minFlowSuccessRate_toTRACK; // percent . from previous frame to the new one.
	int		minFlowSuccessRate_toLEARN; // percent . from previous frame to the new one.

	void consider_duplicates();
	void set_featurePnts_into_image(Point *temp, Mat &targetMask);

	void display_fPoint_4debug(vector<Point2f> newFlowFeatures);

public:
	Target			trackedTarget;	//for use at outside (calling)functions

	Rect			TrackerROI; 

	enum  TRACK_STATE {TRACKER_OFF = 0 , TRACKER_LEARNING, TRACKER_TRACKING} ;
	TRACK_STATE  Tracker_State				= TRACKER_OFF ;

	Rect			current_trackingROI;
	/* original target properties */
	bool			newTargetSituation		= false;
	Rect			OriginalTargetROI;
	Mat				OriginalTarget;
	Mat				grayOriginalTarget;
	vector<Point2f> OriginalTargetFeatures; 	
	double			TrkErrX_Avg				=0;  // average for 3 readings	
	double			TrkErrX_Readings[Nreads] ;
	int				TrkErrX_readIndex		= 0;

	int				TrackPercent				= 0;		// % of the feature points that has correspondance to the next/previous frame
    bool            freshStart					= true;
    Mat_<float>     rigidTransform				= Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix;
	int				min_features				= 5;// depend on light conditions 10;//40//200; 
	int				num_of_maxCornersFeatures	= 100;//300;->100
	float			mid_level_percent			= 1.25;		//	1/1.25=80%
	Size			flowSearchWinSize			=	Size(21,11);	//size(w,h)

	//int		alphaSlider       = 0;
	//int		alphaSlider2      = 0;
	///char	TrackbarName[50]  = "StatusSum";
	///char	TrackbarName2[50] = "StatusSumPercent";

	int		alphaSlider_max		=	100;

	Moments		m; 
	Point		MassCenter;

	/* functions */

	Tracker();
	~Tracker();

	// both imgTarget, imgROI should be of the same size.
	//void processImage(Mat inGray, Mat imgTarget,  Rect Brect);
	void processImage(Mat inputGrayIm, Target *currentTargetMask) ;
};

#endif  // FEATURETRACKER_H