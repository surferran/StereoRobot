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
}

Tracker::~Tracker() 
{
}

/* used when system_status is FOUND_SOME_MOVEMENT only */
void Tracker::setNewTarget(
							Rect			newTargetROI,	// ROI in original image source coor.system
							Mat				newTarget,		// image of target alone. resolution is not known a head
							Rect			TrackingROI)	// tracker ROI to work with. set by the calling method
{
	newTargetSituation	= true;
	//ThresholdTypes//	threshold(newTargetROI, OriginalTargetROI, 0, 9999, THRESH_TOZERO );
	/*if (newTargetROI.x < 0) newTargetROI.x = 0;
	if (newTargetROI.y < 0) newTargetROI.y = 0;*/
	//if (newTargetROI.x + newTargetROI.width > imSource.W) newTargetROI.x = imSource.W;

	//////////////////////////////////

	OriginalTargetROI	= newTargetROI;		// arrives as result of BKgndSubs, therefore should be correct(in limits)
	current_trackingROI	= OriginalTargetROI;

	OriginalTarget		= newTarget.clone();
	TrackerROI			= TrackingROI;

	cvtColor(OriginalTarget, grayOriginalTarget, CV_BGR2GRAY);
	//////////////////////////////////

	prevImProp.relevantROI = newTargetROI ;
	cvtColor(newTarget, prevImProp.grayImage, CV_BGR2GRAY);
	goodFeaturesToTrack(prevImProp.grayImage, prevImProp.goodFeatures, num_of_maxCornersFeatures,0.01,10);	//  int maxCorners, double qualityLevel, double minDistance,
	///cout << "found on new target " << OriginalTargetFeatures.size() << " features\n";		//TODO: set to operate through GUI object..
	 
	/* OriginalTargetFeatures : store feature points in full image coordinates, rather then local image target ones */
	/* trackedFeatures = OriginalTargetFeatures */
	trackedFeatures.clear();
	for (int i = 0; i < prevImProp.goodFeatures.size(); ++i) {
		prevImProp.goodFeatures[i].x = prevImProp.goodFeatures[i].x ;//+ OriginalTargetROI.x;
		prevImProp.goodFeatures[i].y = prevImProp.goodFeatures[i].y ;//+ OriginalTargetROI.y;
		trackedFeatures.push_back(prevImProp.goodFeatures[i]);
	}

	TrackPercent	= 0*100;  // just learning
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

void Tracker::processImage(Mat inputGrayIm, Mat newImage,  StereoRobotApp::SYSTEM_STATUS external_state, Rect Brect) //need mask and need image?.
{
	vector<uchar> flow_output_status; 
	vector<float> flow_output_errors;

///	if (external_state ==  StereoRobotApp::FOUND_SOME_MOVEMENT)
	{
		/* feature tracker part - in this section only learn the potential target */
		// image given from BackSubs so it shows only foreground. in original image size.
		    // new image is a mask from BGSubs & depth
		// later implement full gray image in the 'current' struct
		currentImProp.relevantROI	= Brect;// boundingRect ( newImage );
		cvtColor(newImage, currentImProp.grayImage, CV_BGR2GRAY); 
		goodFeaturesToTrack(currentImProp.grayImage, currentImProp.goodFeatures, num_of_maxCornersFeatures,0.01,10); // can add a mask
		currentImProp.grayImage		= inputGrayIm.clone(); 

		if (Tracker_State == TRACKER_OFF)
		{
			prevImProp		= currentImProp ; 
			Tracker_State	= TRACKER_LEARNING ;	
			return;
		}
		
		if (prevImProp.goodFeatures.size()==0)
			return;		// describe error / lost

		// find connection between 2 images
		vector<Point2f> newFlowFeatures;  
		calcOpticalFlowPyrLK(	prevImProp.grayImage	, currentImProp.grayImage , 
								prevImProp.goodFeatures , newFlowFeatures , 
								flow_output_status, flow_output_errors, flowSearchWinSize, 3,
								TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),0, 0.001);	

		///--currentImProp.goodFeatures + --
		// filter newGoodFeatures
		int features_vec_size ,i ;
		trackedFeatures.clear();

	//	features_vec_size = currentImProp.goodFeatures.size()  ;
	//	for ( i = 0; i < features_vec_size ; ++i) { 
	//		trackedFeatures.push_back(currentImProp.goodFeatures[i]);
	//	} 
		features_vec_size = newFlowFeatures.size() ; 
		for ( i = 0; i < features_vec_size ; ++i) { 
			if (flow_output_status[i])
				trackedFeatures.push_back(newFlowFeatures[i]);
		} 

		int 	diffSizes 	= newFlowFeatures.size() - trackedFeatures.size() ;
		double 	successRate = (double)diffSizes / (double)newFlowFeatures.size() * 100.0 ;  //[%]
		if ( successRate > 30 )
		{
			///if ( successRate >20 )  trackedFeatures.size()>5?
			{
				// replace current feature points to enlarg the tracked vector
				trackedFeatures.clear();
				features_vec_size = currentImProp.goodFeatures.size() ;
				for ( i = 0; i < features_vec_size ; ++i) {
						trackedFeatures.push_back(currentImProp.goodFeatures[i]);
				}
			}
			if (external_state == StereoRobotApp::FOUND_SOME_MOVEMENT)
				if ( successRate >20 ){
					Tracker_State = TRACKER_TRACKING;
					system_state = StereoRobotApp::FOUND_GOOD_TARGET ;
				}
		}
		else
			//if ( successRate <20 )
			{
				trackedFeatures.clear();
				MassCenter		= Point(inputGrayIm.size().width/2, inputGrayIm.size().height/2);
				Tracker_State 	= TRACKER_OFF;
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
		Mat tmpIM = newImage.clone();
		int r	= 2;	//3
		for(  i = 0; i < trackedFeatures.size(); i++ )
		{ 
			circle( tmpIM, trackedFeatures[i], r, 				Scalar(255, 100, 255), -1, 8, 0 );
		}	 
		for(  i = 0; i < currentImProp.goodFeatures.size(); i++ )
		{ 
			circle( tmpIM, currentImProp.goodFeatures[i], r, 	Scalar(10, 100, 255), -1, 8, 0 );
		}	
		for(  i = 0; i < newFlowFeatures.size(); i++ )
		{ 
			circle( tmpIM, newFlowFeatures[i], r, 				Scalar(10, 255, 255), -1, 8, 0 );
		}	 
		imshow ("debug summarized features"	, tmpIM);


		prevImProp.goodFeatures		= trackedFeatures;  //currentImProp ; 
		prevImProp.grayImage		= currentImProp.grayImage;
		prevImProp.relevantROI		= boundingRect(trackedFeatures);

		// check condition for GOOD_TRACKING. instead of by subs. 
		Target::TargetState tmpTargStat = Target_obj.calc_target_properties(tmp2) ;
		if (tmpTargStat==Target::Target_present)
			//system_state = TRACKING_GOOD_QUALITY_TARGET ;
			system_state = StereoRobotApp::FOUND_GOOD_TARGET ;

	}
		return;

}


