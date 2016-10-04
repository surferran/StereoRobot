#ifdef COMPILING_ON_ROBOT
#include "FeatureTracker.hpp"
#include "myGUI_handler.h"
#else
#include "..\Headers\FeatureTracker.hpp"
#include "..\Headers\myGUI_handler.h"
#include "..\Headers\Target.hpp"

#include "..\imporeted_raw_code_examples/MatchTemplate_Demo.cpp"
#include "..\imporeted_raw_code_examples/detect_blob.cpp"
#endif


extern StereoRobotApp::SYSTEM_STATUS	system_state ;

Tracker::Tracker() 
{
	/* for the good features to track function */
	GFTFquality			=	0.01;
	GFTFminDistance		=	10.0;
	/* threshold conditions for going through track stages */
	minFlowSuccessRate_toLEARN	=	90;
	minROIareaRatio_toLEARN		=	10;
	minFPsize_toLEARN			=	30;
	minFlowSuccessRate_toTRACK	=	30;
	minROIareaRatio_toTRACK		=	 5;
	minFPsize_toTRACK			=	 5;
}

Tracker::~Tracker() 
{
}

/////////////////////////////

// Lexicographic compare, same as for ordering words in a dictionnary:
// test first 'letter of the word' (x coordinate), if same, test 
// second 'letter' (y coordinate).
bool lexico_compare(const Point2f& p1, const Point2f& p2) {
	if(p1.x < p2.x) { return true; }
	if(p1.x > p2.x) { return false; }
	return (p1.y < p2.y);
}


bool points_are_equal(const Point2f& p1, const Point2f& p2) {
	return ((p1.x == p2.x) && (p1.y == p2.y));
}

/////////////////////////////

void Tracker::processImage(Mat inputGrayIm, Mat newImageMask, Rect Brect) //need mask and need image?.
{
	// newImageMask is a mask from BGSubs & depth
	// Brect - is boundingRect ( newImageMask );

	static int		learnTRKcounter		=	0;
	vector<uchar>	flow_output_status; 
	vector<float>	flow_output_errors;
	Mat				currentMask;
	double			tmpRatio;
	vector<Point2f> newFlowFeatures;  

	currentImProp.relevantROI	= Brect;	
	currentImProp.grayImage		= inputGrayIm.clone(); 
	currentMask					= newImageMask.clone();

	goodFeaturesToTrack(currentImProp.grayImage, currentImProp.goodFeatures, 
							num_of_maxCornersFeatures, GFTFquality, GFTFminDistance, currentMask );  

	//TODO?: if (currentImProp.goodFeatures.size() < min..) .. alert

	if (Tracker_State == TRACKER_OFF)
	{
		learnTRKcounter	= 0;
		//  verify anough feature points found. and ROI in relevant size for possible target.
		tmpRatio = 100.0 * (double) currentImProp.relevantROI.area() /  (double) currentImProp.grayImage.size().area() ;
		if (tmpRatio > minROIareaRatio_toLEARN)	// can pass this condition to above. to 'save' a run-cycle.
			if (currentImProp.goodFeatures.size() > minFPsize_toLEARN)
			{
				Tracker_State	= TRACKER_LEARNING ;/* in this section only learn the potential target */	
				prevImProp		= currentImProp ; 
			}
		return;
	}
		
	if (prevImProp.goodFeatures.size()==0)
	{
		prevImProp		= currentImProp ;
		return;		// describe error / lost	?
	}

	/* Tracker_State	= TRACKER_LEARNING or TRACKER_TRACKING */

	// find connection(flow) between 2 images
	newFlowFeatures = currentImProp.goodFeatures;	// initial guess
	calcOpticalFlowPyrLK(	prevImProp.grayImage	, currentImProp.grayImage , 
							prevImProp.goodFeatures , newFlowFeatures , 
							flow_output_status, flow_output_errors, flowSearchWinSize, 3,	// 3 is maxLevel for pyramids
							TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),0, 0.001);	
		
	/* build the vector trackedFeatures */
	// add to, and filter newGoodFeatures
	int features_vec_size ,i ;
	trackedFeatures.clear();

	features_vec_size = newFlowFeatures.size() ; 
	for ( i = 0; i < features_vec_size ; ++i) { 
		if (flow_output_status[i])
			trackedFeatures.push_back(newFlowFeatures[i]);
	} 

	// measure the flow calculation success rate 
	int 	diffSizes 	= newFlowFeatures.size() - trackedFeatures.size() ;
	double 	successRate = (1. - (double)diffSizes / (double)newFlowFeatures.size()) * 100.0 ;  //[%]

	// when LEARNING : add 'blindly' the current image feature points. for the new current ROI.
	if (Tracker_State == TRACKER_LEARNING)	
	{
		features_vec_size = currentImProp.goodFeatures.size()  ;
		for ( i = 0; i < features_vec_size ; ++i) { 
			trackedFeatures.push_back(currentImProp.goodFeatures[i]);
		}
	}

	/******************************************/

	if ( Tracker_State == TRACKER_LEARNING)
	{
		if ( successRate > minFlowSuccessRate_toLEARN )
		{
			learnTRKcounter++;
			if (learnTRKcounter > 3)
				Tracker_State = TRACKER_TRACKING;
		}
		else
			;	//TODO: need to check how many gapped frames (with not enough features)
	}
	else // state is TRACKER_TRACKING:
	{
		//tmpRatio = 100.0 * (double) currentImProp.relevantROI.area() /  (double) currentImProp.grayImage.size().area() ;
		tmpRatio = 100.0 * (double) boundingRect(trackedFeatures).area() /  (double) currentImProp.grayImage.size().area() ;
		if ( ( successRate < minFlowSuccessRate_toTRACK )	
				|| (trackedFeatures.size() < minFPsize_toTRACK) 
				|| (tmpRatio < minROIareaRatio_toTRACK) )
		{
			Tracker_State	=	TRACKER_OFF;	// TODO: also cout a message?
			return ;
		}
	}

	// sort and delete duplicates
	// ( ref by : http://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector )
	//			->	http://stackoverflow.com/questions/25197805/how-to-delete-repeating-coordinates-of-vectorpoint2f 
	///sort( trackedFeatures.begin(), trackedFeatures.end(), lexico_compare );		
	///trackedFeatures.erase( unique( trackedFeatures.begin(), trackedFeatures.end() , points_are_equal), trackedFeatures.end() );

	/* calc center of points, and trackErr.x (bearing)  */
	m	= moments(trackedFeatures,false);					// points moment 
	Mat tmp2 = Mat::zeros(inputGrayIm.size(), CV_8U);
	for (int i = 0; i < trackedFeatures.size(); ++i) {
		{
			int xx = trackedFeatures[i].x;
			int yy = trackedFeatures[i].y;
			if ( (xx<tmp2.size().width) &&
				(yy<tmp2.size().height) &&
				(xx>0 && yy>0) 
				)
			{
				auto ptr = tmp2.ptr<uchar>(yy) ;
				ptr[xx] = 1*255;
			}
		}
	}
	m	= moments(tmp2,true); 
	MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
	// ..tmp2 Mat can be showm as an image. for debug..
		
	//////
	Mat tmpIM = newImageMask.clone();

	cv::cvtColor(tmpIM , tmpIM  , CV_GRAY2BGR);
	int r	= 2;	//3
	for(  i = 0; i < trackedFeatures.size(); i++ )
	{ 
		circle( tmpIM, trackedFeatures[i], r, 				Scalar(255, 100, 255), -1, 8, 0 );//pink
	}	 
	for(  i = 0; i < currentImProp.goodFeatures.size(); i++ )
	{ 
		circle( tmpIM, currentImProp.goodFeatures[i], r, 	Scalar(10, 100, 255), -1, 8, 0 );//orange
	}	
	for(  i = 0; i < newFlowFeatures.size(); i++ )
	{ 
		circle( tmpIM, newFlowFeatures[i], r, 				Scalar(10, 255, 255), -1, 8, 0 );//yellow
	}	 /**/
	imshow ("debug summarized features"	, tmpIM);

	// set data for next cycle loop
	prevImProp.goodFeatures		= trackedFeatures; ; 
	prevImProp.grayImage		= currentImProp.grayImage;
	prevImProp.relevantROI		= Rect();//boundingRect(trackedFeatures);	//this one is just for reference. not useful

	// set target calculated properties, by resultant faeture points mask. 
	trackedTarget.calc_target_mask_properties(tmp2) ; 
	
	return;

}


