#ifdef COMPILING_ON_ROBOT
#include "FeatureTracker.hpp"
#include "myGUI_handler.h"
#else
#include "..\Headers\FeatureTracker.hpp"
#include "..\Headers\myGUI_handler.h"
#include "..\Headers\Target.hpp"
//
//#include "..\imporeted_raw_code_examples/MatchTemplate_Demo.cpp"
//#include "..\imporeted_raw_code_examples/detect_blob.cpp"
#endif


extern StereoRobotApp::SYSTEM_STATUS	system_state ;
extern	myGUI_handler myGUI;

Tracker::Tracker() 
{
	/* for the good features to track function */
	GFTFquality			=	0.005;//	0.01;
	GFTFminDistance		=	6;//10.0;
	/* threshold conditions for going through track stages */
	minFlowSuccessRate_toLEARN	=	60;//90;
	minROIareaRatio_toLEARN		=	 5;
	minFPsize_toLEARN			=	10;
	minFlowSuccessRate_toTRACK	=	20;//30 //new..
	minROIareaRatio_toTRACK		=	 3;//5;//1.5
	minFPsize_toTRACK			=	 7;
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
void Tracker::consider_duplicates()
{
	// sort and delete duplicates
	// ( ref by : http://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector )
	//			->	http://stackoverflow.com/questions/25197805/how-to-delete-repeating-coordinates-of-vectorpoint2f 
	///sort( trackedFeatures.begin(), trackedFeatures.end(), lexico_compare );		
	///trackedFeatures.erase( unique( trackedFeatures.begin(), trackedFeatures.end() , points_are_equal), trackedFeatures.end() );
}
void Tracker::set_featurePnts_into_image(Point *returnMassCenter, Mat &targetMask)
{
	//TODO: cut by last tmpROI !!

	/* calc center of points, and trackErr.x (bearing will be atan(x/D) )  */
	///m	= moments(trackedFeatures,false);					// points moment 
	Mat tmp2 = Mat::zeros(currentImProp.grayImage.size(), CV_8U);
	for (int i = 0; i < trackedFeatures.size(); ++i) {
		{
			int xx = trackedFeatures[i].x;
			int yy = trackedFeatures[i].y;
			if ( (xx < tmp2.size().width) &&
				 (yy < tmp2.size().height) &&
				 (xx > 0) && (yy > 0) 
				)
			{
				auto ptr = tmp2.ptr<uchar>(yy) ;
				ptr[xx] = 1*255;
			}
		}
	}
	m	= moments(tmp2,false); 
	///m	= moments(tmp2,true);	//trit binary image - all 0 or 1. i need with 255 values..
	*returnMassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
	targetMask = tmp2;
													// ..tmp2 Mat can be showm as an image. for debug..
}
/////////////////////////////

//void Tracker::processImage(Mat inputGrayIm, Mat newImageMask, Rect Brect) //need mask and need image?.
void Tracker::processImage(Mat inputGrayIm, Target *mainTarget) 
{
	// newImageMask is a mask from BGSubs & depth
	// Brect - is boundingRect ( newImageMask );

	static int		learnTRKcounter		=	0;
	static int 		trackGapFrameCounter = 0;	// added new
	vector<uchar>	flow_output_status; 
	vector<float>	flow_output_errors;
	Mat				currentMask;
	double			tmpRatio;
	vector<Point2f> newFlowFeatures,
					newFlowFeaturesBack;  
	Target			currentTarget = *mainTarget;

	currentImProp.grayImage		= inputGrayIm.clone(); 
	currentImProp.relevantROI	= currentTarget.target_mask_prop.boundRect ;  //Brect;	
	currentMask					= currentTarget.target_mask_prop.maskIm.clone();
	/* verify if empty (all black) mask */
	if (currentTarget.target_mask_prop.boundAreaRatio==0)	//TODO:can put this condition earlier in the main app loop 
	{
		trackGapFrameCounter++;				// TODO: if above 5 - define low quality target. - make yellow frame for the display..
		if (trackGapFrameCounter > 25)	
			Tracker_State = TRACKER_OFF;
		//trackGapFrameCounter = 0;
		return;
	}

	// tmpROI is MASK in potential area of new target. 
	// potential target is Trimmed area according to ROI (from Depth or BgSubt)	
	Mat tmpROI							=  Mat::zeros( currentImProp.grayImage.size() , currentImProp.grayImage.type() );
	tmpROI( currentImProp.relevantROI )	= 255;				
	inputGrayIm.copyTo(currentTarget.potential_target , tmpROI);	//not used..

	int an=2;	//an=1->kernel of 3
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(an*2+1, an*2+1), Point(an, an) );
	currentMask = tmpROI;   // take the rectangle , not just the real insides		//new addition! 10:59
	dilate		(currentMask,	currentMask, element);  // inflate the region to search in 
	goodFeaturesToTrack(currentImProp.grayImage, currentImProp.goodFeaturesCoor, 
							num_of_maxCornersFeatures, GFTFquality, GFTFminDistance, currentMask );  

	if ( (currentImProp.goodFeaturesCoor.size() <= 1) ||
			(prevImProp.goodFeaturesCoor.size() <= 1) )
	{
		trackGapFrameCounter++;
		if (trackGapFrameCounter > 25)
			Tracker_State 	= TRACKER_OFF;
		//trackGapFrameCounter = 0;
		prevImProp		= currentImProp ;
		return;
	}

	if (Tracker_State == TRACKER_OFF)
	{
		learnTRKcounter	= 0;
		trackGapFrameCounter = 0;
		if (currentImProp.goodFeaturesCoor.size() > minFPsize_toLEARN)
		{
			//  verify anough feature points found. and ROI in relevant size for possible target.
			double tmpArea1 = (double) currentImProp.relevantROI.area();
			double tmpArea2 = (double) currentImProp.grayImage.size().area();
			tmpRatio = 100.0 * tmpArea1 / tmpArea2 ;
			if (tmpRatio > minROIareaRatio_toLEARN)	// can pass this condition to above. to 'save' a run-cycle.
				{
					Tracker_State	= TRACKER_LEARNING ;/* in this section only learn the potential target */	
					prevImProp		= currentImProp ; 
				}
		}
		return;
	}
		
	///* do not allow tracking trial without minimum number of feature points, every frame.
	//     because frame rate is not high, don't tollarance frame gaps. */
	//if (prevImProp.goodFeaturesCoor.size() </*=*/ minFPsize_toTRACK)
	//{
	//	prevImProp		= currentImProp ;
	//	return;		// describe error / lost / set alarm	?
	//}

	/* now Tracker_State is TRACKER_LEARNING or TRACKER_TRACKING */

	// find connection(flow) between 2 images
	newFlowFeatures = currentImProp.goodFeaturesCoor;	// initial guess
	calcOpticalFlowPyrLK(	prevImProp.grayImage	, currentImProp.grayImage , 
							prevImProp.goodFeaturesCoor , newFlowFeatures , 
							flow_output_status, flow_output_errors, flowSearchWinSize, 3,	// 3 is maxLevel for pyramids
							TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),0, 0.001);	
		
	/* build the vector trackedFeatures */
	// add to, and filter newGoodFeatures
	int features_vec_size ,i ;
	trackedFeatures.clear();

	/* roi adjustment */
	Rect	roi = currentImProp.relevantROI ;
	double	roi_extraction_factor = 0.1 ;		 // a fraction addition to limits
	roi.x      *= 1. - roi_extraction_factor ;
	roi.y      *= 1. - roi_extraction_factor ; 
	if (roi.x < 0) roi.x = 0;
	if (roi.y < 0) roi.y = 0;
	//	assuming original given ROI is within image limits.
	if ( ( roi.x + roi.width )*(1. + roi_extraction_factor) < currentImProp.grayImage.size().width )
		roi.width  *= 1. + roi_extraction_factor ;
	if ( ( roi.y + roi.height )*(1. + roi_extraction_factor) < currentImProp.grayImage.size().height )
		roi.height  *= 1. + roi_extraction_factor ;

	features_vec_size = newFlowFeatures.size() ; 
	for ( i = 0; i < features_vec_size ; ++i) { 
		if (flow_output_status[i])
		{
			/* check also for staying in the ROI of the given mask (or near) */
			int xx   = newFlowFeatures[i].x;
			int yy   = newFlowFeatures[i].y;
			if ( ( roi.x <= xx ) && ( xx <= (roi.x+roi.width) ) &&
				 ( roi.y <= yy ) && ( yy <= (roi.y+roi.height) )
				)
				trackedFeatures.push_back(newFlowFeatures[i]);
		}
	} 

	//// option for checking backwards flow
	///*********/	/*********/	/*********/	/*********/
	//newFlowFeaturesBack = prevImProp.goodFeaturesCoor;	// initial guess
	//calcOpticalFlowPyrLK(	currentImProp.grayImage , prevImProp.grayImage	,
	//	trackedFeatures			, newFlowFeaturesBack , 
	//	flow_output_status, flow_output_errors, flowSearchWinSize, 3,	// 3 is maxLevel for pyramids
	//	TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),0, 0.001);

	//trackedFeaturesBack.clear();
	//features_vec_size = newFlowFeaturesBack.size() ; 
	//for ( i = 0; i < features_vec_size ; ++i) { 
	//	if (flow_output_status[i])
	//		trackedFeaturesBack.push_back(newFlowFeaturesBack[i]);
	//} 
	///*********/	/*********/	/*********/	/*********/

	// measure the flow calculation success rate 

	//int 	diffSizesBack 	= newFlowFeatures.size() - newFlowFeaturesBack.size() ;	//before screening
	//int 	diffSizesBack2 	= trackedFeatures.size() - trackedFeaturesBack.size() ;	//after screening

	int 	diffSizes 	= newFlowFeatures.size() - trackedFeatures.size() ;
	double 	successRate = (1. - (double)diffSizes / (double)newFlowFeatures.size()) * 100.0 ;  //[%]

	// when LEARNING : add 'blindly' the current image feature points. for the new current ROI.
	//if (Tracker_State == TRACKER_LEARNING)	
	//{
	//	if ( trackedFeatures.size() >= minFPsize_toLEARN )
	//	{
	//		features_vec_size = currentImProp.goodFeaturesCoor.size()  ;
	//		for ( i = 0; i < features_vec_size ; ++i) { 
	//			trackedFeatures.push_back(currentImProp.goodFeaturesCoor[i]);
	//		}
	//	}
	//	else
	//		;//error//break learning, go back stdby..  TODO..
	//}

	if ( ( (Tracker_State == TRACKER_LEARNING) && ( trackedFeatures.size() >= minFPsize_toLEARN ) )
		 ||
	 	 ( (Tracker_State == TRACKER_TRACKING) && ( trackedFeatures.size() < 7*minFPsize_toTRACK ) // 3*7 to 100 ->~ 49
			 && ( trackedFeatures.size() > minFPsize_toTRACK ) ) )
	{
		features_vec_size = currentImProp.goodFeaturesCoor.size()  ;
		for ( i = 0; i < features_vec_size ; ++i) { 
			trackedFeatures.push_back(currentImProp.goodFeaturesCoor[i]);
		}
	}
	/*******************got trackedFeatures that are checked ***********************/

	if ( Tracker_State == TRACKER_LEARNING)
	{
		if ( ( successRate > minFlowSuccessRate_toLEARN ) && ( trackedFeatures.size() >= minFPsize_toLEARN ) )
		{
			learnTRKcounter++;
			if (learnTRKcounter > 1)	// 1 as minCycleFromLearnToTrack
				Tracker_State = TRACKER_TRACKING;
		}
		else
			learnTRKcounter = 0;
			//;	//TODO: need to check how many gapped frames (with not enough features)
	}
	else // state is TRACKER_TRACKING:
	{
		double tmpArea1 = (double) boundingRect(trackedFeatures).area();
		double tmpArea2 = (double) currentImProp.grayImage.size().area();
		tmpRatio = 100.0 * tmpArea1 / tmpArea2 ;
		if ( ( successRate < minFlowSuccessRate_toTRACK )	
				|| (trackedFeatures.size() < minFPsize_toTRACK) 
				|| (tmpRatio < minROIareaRatio_toTRACK) )
		{
			trackGapFrameCounter++;
			if (trackGapFrameCounter > 25)
			{
				Tracker_State	= TRACKER_OFF;	// optional TODO: also cout a message?
			}
			prevImProp		= currentImProp ;	// this maybe not needed
			return ;
		}
		else
			trackGapFrameCounter = 0;
	}

	///consider_duplicates();

	Mat tmp2;
	/* embed 'trackedFeatures' into matrix tmp2 */
	set_featurePnts_into_image(&MassCenter, tmp2);		
	// set target calculated properties, by resultant faeture points mask. 

	trackedTarget  = *mainTarget ;///
	trackedTarget.set_target_mask_properties(tmp2) ; 
	//	trackedTarget.target_object_prop.target_estimated_distance	=	0;//
	trackedTarget.target_object_prop.target_estimated_dx	=	trackedTarget.target_mask_prop.MassCenter.x - 
																 trackedTarget.target_mask_prop.image_mask_size_width / 2.;
////#ifndef COMPILING_ON_ROBOT
	display_allFPoints(true, newFlowFeatures);
//#endif

	// set data for next cycle loop
	prevImProp.goodFeaturesCoor	= trackedFeatures; ; 
	prevImProp.grayImage		= currentImProp.grayImage;
	prevImProp.relevantROI		= Rect();//boundingRect(trackedFeatures);	//this one is just for reference. not useful
	 
	*mainTarget = trackedTarget;

	return;

}

void Tracker::display_allFPoints(bool forDebug, vector<Point2f> newFlowFeatures)
{
	Mat tmpIM = trackedTarget.target_mask_prop.maskIm.clone();//  newImageMask.clone();

	cv::cvtColor(tmpIM , tmpIM  , CV_GRAY2BGR);
	int r	= 3;
	int i;
	if (forDebug)
	{
		for(  i = 0; i < currentImProp.goodFeaturesCoor.size(); i++ )
		{ circle( tmpIM, currentImProp.goodFeaturesCoor[i], r, 	Scalar(10, 100, 255), -1, 8, 0 );//orange
		}	
		for(  i = 0; i < newFlowFeatures.size(); i++ )
		{ circle( tmpIM, newFlowFeatures[i], r, 				Scalar(10, 255, 255), -1, 8, 0 );//yellow
		}	
	}
	r=2;
	for(  i = 0; i < trackedFeatures.size(); i++ )
	{ circle( tmpIM, trackedFeatures[i], r, 				Scalar(255, 100, 255), -1, 8, 0 );//pink
	}	
	
	// TODO: add tmpIM the successRate (add variable to the header)  new 11:02
	myGUI.plotImages[myGUI_handler::WIN5_NDX_FeaturePoints]	=	tmpIM;
	if (!myGUI.bSHOW_as_demo_movie_flow)
		imshow ( myGUI.plotWindowsNames[myGUI_handler::WIN5_NDX_FeaturePoints]	, tmpIM);
}

