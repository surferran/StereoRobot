#include "FeatureTracker.hpp"
#include "myGUI_handler.h"

#include "imporeted_raw_code_examples/MatchTemplate_Demo.cpp"

extern SYSTEM_STATUS	system_state ;
extern myGUI_handler	myGUI;

///Tracker::Tracker() {}


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
	
	OriginalTargetROI	= newTargetROI;		// arrives as result of BKgndSubs, therefore should be correct(in limits)
	current_trackingROI	= OriginalTargetROI;
	
	OriginalTarget		= newTarget.clone();
	TrackerROI			= TrackingROI;

	cvtColor(OriginalTarget, grayOriginalTarget, CV_BGR2GRAY);
	goodFeaturesToTrack(grayOriginalTarget, OriginalTargetFeatures, num_of_maxCornersFeatures,0.01,10);	//  int maxCorners, double qualityLevel, double minDistance,
	cout << "found on new target " << OriginalTargetFeatures.size() << " features\n";		//TODO: set to operate through GUI object..
	 
	/* OriginalTargetFeatures : store feature points in full image coordinates, rather then local image target ones */
	/* trackedFeatures = OriginalTargetFeatures */
	trackedFeatures.clear();
	for (int i = 0; i < OriginalTargetFeatures.size(); ++i) {
		OriginalTargetFeatures[i].x = OriginalTargetFeatures[i].x + OriginalTargetROI.x;
		OriginalTargetFeatures[i].y = OriginalTargetFeatures[i].y + OriginalTargetROI.y;
		trackedFeatures.push_back(OriginalTargetFeatures[i]);
	}

	TrackPercent	= 100;
}

// getting newIage variable, and saving it as Gray of previous or current frame.
// trackedFeatures vector is usually the points from previous frame . 
//		newly calculated or given as result of previous 'opticalFlow'.
void Tracker::processImage(Mat newImage,  SYSTEM_STATUS external_state) 
{
	Mat grayROI ;

	vector<uchar> flow_output_status; 
	vector<float> flow_output_errors;

	if (newTargetSituation==true)
	{
		newTargetSituation = false;
		cvtColor(newImage.clone()   ,grayROI   ,CV_BGR2GRAY);
		grayROI.copyTo(prevGrayROI);

		return;
	}

	cvtColor(newImage   ,grayROI   ,CV_BGR2GRAY);

    vector<Point2f> corners;  

	/* check for lost of minimum feature points to track */
	if ( (trackedFeatures.size() < min_features /** mid_level_percent*/ ) )		//TODO: move this condition to the end?
	{
		// when near minimum limit - find more feature in ROI around those still alive
		////set LOW_QUALITY_TRACKING

		// if bellow minimum - anounce loss of tracking. and stop..
		cout << "loss of tracking fetures....!"<<endl;

		TrackPercent	= 0;
		return;
	}

    if(!prevGrayROI.empty()) 
	{
		Mat tmpIm;

		// check given ROI boundaries. 
		if (current_trackingROI.x < 0) current_trackingROI.x = 0;
		if (current_trackingROI.y < 0) current_trackingROI.y = 0;
		if (current_trackingROI.x + current_trackingROI.width > prevGrayROI.size().width) 
			current_trackingROI.width = prevGrayROI.size().width - current_trackingROI.x ;
		if (current_trackingROI.y + current_trackingROI.height > prevGrayROI.size().height) 
			current_trackingROI.height = prevGrayROI.size().height - current_trackingROI.y ;

#define COMPARE_TO_ORIGINAL_TARGET false
		if ( ! COMPARE_TO_ORIGINAL_TARGET )
		{
			// take relevant ROI out of the previous whole image
			// calculate newly feature points vector
			prevGrayROI(current_trackingROI).copyTo(tmpIm);
			goodFeaturesToTrack(tmpIm, corners, num_of_maxCornersFeatures,0.01,10);	//  int maxCorners, double qualityLevel, double minDistance,
			//cout << "(re-)found " << corners.size() << " features\n";
		 ///release(tmpIm);??

			// set corners -> trackedFeatures 
			trackedFeatures.clear();
			for (int i = 0; i < corners.size(); ++i) {
				/* store feature points in full image coordinates */
				corners[i].x = corners[i].x + current_trackingROI.x;
				corners[i].y = corners[i].y + current_trackingROI.y;
				trackedFeatures.push_back(corners[i]);
			}
		}
		else
		{
			trackedFeatures = OriginalTargetFeatures;	//in image coordinates
		}

		// new input is trackedFeatures ->
		// new output is corners
		// status of 1 means correspondence found. 0 otherwise.
		if ( ! COMPARE_TO_ORIGINAL_TARGET )
			calcOpticalFlowPyrLK(prevGrayROI,grayROI,trackedFeatures,corners,flow_output_status, flow_output_errors,Size(21,11), 3,
				TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),0, 0.001);	//0.1 is too much for low visibility 
		else
		{
			Mat tmpOrig = Mat::zeros( grayROI.size(), grayROI.type() );
			tmpOrig(OriginalTargetROI) = grayOriginalTarget ;

			main_MatchTemplate(grayROI, grayOriginalTarget);	//Match from example code

			/*calcOpticalFlowPyrLK(tmpOrig ,grayROI,trackedFeatures,corners,status,errors,Size(33,33), 3,
				TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),0, 0.001);*/	
		}

		/*////////// display the found feature points from	Prev (prevGrayROI, trackedFeatures)  to 
															Current (grayROI, corners)				 /////////*/

		////
		myGUI.dispFlowChanges(prevGrayROI, trackedFeatures, grayROI, corners, flow_output_status); // TODO: running away the memory!

		alphaSlider  = countNonZero(flow_output_status);
		alphaSlider2 = ((double)countNonZero(flow_output_status))/ ((double)flow_output_status.size()) * 100.0;
		//////////////////////////////////
		if (alphaSlider2<5)
			cout << "alphaSlider2 "<<alphaSlider2 <<"\n";
        /*if(alphaSlider * mid_level_percent < status.size()) */
		freshStart = false;
		if(alphaSlider  < min_features * mid_level_percent) 
		{
            cout << "cataclysmic error . alphaSlider LOW \n";
            rigidTransform	= Mat::eye(3,3,CV_32FC1);	//same as above specific function
            trackedFeatures	.clear();
            prevGrayROI		.release();
            freshStart		= true;

			TrackPercent = 0;
            return;
        } 

		// get last differential movement
        Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,corners,false); 
		if (!newRigidTransform.empty())
		{
			float a = newRigidTransform.at<float>(0,0);
			float b = newRigidTransform.at<float>(0,1);
			float c = newRigidTransform.at<float>(1,0);
			float d = newRigidTransform.at<float>(1,1);
			
			float sX ;
			float sign_a = a>=0? 1:(-1);
			sX = sign_a * sqrtf(powf(a,2) + powf(b,2));
			float sY;
			float sign_d = d>=0? 1:(-1);
			sY = sign_d * sqrt(powf(c,2) + powf(d,2));

			// reduce the effect of scale.
			float* tmp = NULL;
			tmp =  newRigidTransform.ptr<float>(0);
			tmp[0] = tmp[0]/sX;
			tmp[1] = tmp[1]/sX;
			tmp =  newRigidTransform.ptr<float>(1);
			tmp[0] = tmp[0]/sY;
			tmp[1] = tmp[1]/sY;
			
			Mat_<float> nrt33 = Mat_<float>::eye(3,3);
			newRigidTransform.copyTo(nrt33.rowRange(0,2));
			rigidTransform *= nrt33;			

			////////////////////////////////

				//TODO: consier put this before this 'if'
			trackedFeatures.clear();
			for (int i = 0; i < flow_output_status.size(); ++i) {
				if(flow_output_status[i]) {
					trackedFeatures.push_back(corners[i]);
				}
			}
			/* calc center of points, and trackErr.x (bearing)  */
			///Rect newFeaturesRect = 
			Moments		m; 

			m					= moments(trackedFeatures,false);				// points moment 
			Mat tmp2 = Mat::zeros(grayROI.size(), CV_8U);
			for (int i = 0; i < trackedFeatures.size(); ++i) {
				{
					int xx = trackedFeatures[i].x;
					int yy = trackedFeatures[i].y;
					if ( (xx<tmp2.size().width) &&
						 (yy<tmp2.size().height) &&
						(xx>0 && yy>0) ){
								auto ptr = tmp2.ptr<uchar>(yy) ;
											ptr[xx] = 1;
					}
				}
			}
			m = moments(tmp2,true); 
			Point MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
			////RAN//TODO check: current_trackingROI	= boundingRect( trackedFeatures ); 	 //trial: OriginalTargetROI;// 

			int addedSpace = 0*  2;

			// in order to increase next search area
			current_trackingROI.x -= addedSpace; //TODO: change to *1.05 as 5% increase.. and check for staying in image limits
			current_trackingROI.y -= addedSpace;
			current_trackingROI.width  += addedSpace;
			current_trackingROI.height += addedSpace;

			if (current_trackingROI.x < 0) current_trackingROI.x = 0;
			if (current_trackingROI.y < 0) current_trackingROI.y = 0;
			if (current_trackingROI.x + current_trackingROI.width > prevGrayROI.size().width) 
				current_trackingROI.width = prevGrayROI.size().width - current_trackingROI.x ;//-addedSpace;
			if (current_trackingROI.y + current_trackingROI.height > prevGrayROI.size().height) 
				current_trackingROI.height = prevGrayROI.size().height - current_trackingROI.y ;//-addedSpace;



			myGUI.show_graphics_with_image(prevGrayROI, MassCenter, 0, current_trackingROI,
				0, 0, 0,0);
			 

			TrkErrX_Readings[TrkErrX_readIndex] = MassCenter.x - prevGrayROI.size().width/2.0 ;
			TrkErrX_Avg = 0;
			int iNdx=0;
			for ( iNdx=0; iNdx< Nreads; iNdx++)
				if (TrkErrX_Readings[iNdx]!=0.0)			//-1 didnt work..
					TrkErrX_Avg += TrkErrX_Readings[iNdx];
				else
					break;
			TrkErrX_Avg = TrkErrX_Avg / iNdx;
			TrkErrX_readIndex++;
			if (TrkErrX_readIndex>=Nreads) TrkErrX_readIndex = 0;
			
			TrackPercent = alphaSlider2 ;
		}
		else
		{
			cout << "cataclysmic error 2! \n";
			rigidTransform	= Mat::eye(3,3,CV_32FC1);	//same as above specific function
			trackedFeatures	.clear();
			prevGrayROI		.release();
			freshStart		= true;
			TrackPercent	= 0;
			return;
		}
    }
	grayROI.copyTo(prevGrayROI);
}
