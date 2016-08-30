#include "FeatureTracker.hpp"

extern SYSTEM_STATUS	system_state ;

Tracker::Tracker():freshStart(true) {
    rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
}


void Tracker::setNewTarget(
							Rect			newTargetROI,	// ROI in original image source coor.system
							Mat				newTarget,		// image of target alone. resolution is not known a head
							Rect			TrackingROI)	// tracker ROI to work with. set by the calling method
{
	newTargetSituation	= true;
	//ThresholdTypes//	threshold(newTargetROI, OriginalTargetROI, 0, 9999, THRESH_TOZERO );
	if (newTargetROI.x < 0) newTargetROI.x = 0;
	if (newTargetROI.y < 0) newTargetROI.y = 0;
	//if (newTargetROI.x + newTargetROI.width > imSource.W) newTargetROI.x = imSource.W;
	
	OriginalTargetROI	= newTargetROI;
	current_trackingROI	= OriginalTargetROI;
	
	OriginalTarget		= newTarget.clone();
	TrackerROI			= TrackingROI;

	Mat grayTarget; cvtColor(OriginalTarget, grayTarget, CV_BGR2GRAY);
	goodFeaturesToTrack(grayTarget, TargetFeatures, num_of_maxCornersFeatures,0.01,10);	//  int maxCorners, double qualityLevel, double minDistance,
	cout << "found on new target " << TargetFeatures.size() << " features\n";

	// set corners -> trackedFeatures 

	trackedFeatures.clear();
	for (int i = 0; i < TargetFeatures.size(); ++i) {
		/* store feature points in full image coordinates, rather then local image target ones */
		TargetFeatures[i].x = TargetFeatures[i].x + OriginalTargetROI.x;
		TargetFeatures[i].y = TargetFeatures[i].y + OriginalTargetROI.y;
		trackedFeatures.push_back(TargetFeatures[i]);
	}

	TrackPercent	= 100;
}

// both imgTarget, imgROI should be of the same size.
void Tracker::processImage(Mat newImage,  SYSTEM_STATUS external_state) 
{

	//RNG rng(12345);//RANDV
	Mat grayROI ;
	if (newTargetSituation==true)
	{
		newTargetSituation = false;
		   cvtColor(newImage.clone()   ,grayROI   ,CV_BGR2GRAY);
		grayROI.copyTo(prevGrayROI);

		return;
	}

	///Mat grayTarget; cvtColor(imgTarget,grayTarget,CV_BGR2GRAY);	// can be removed, because i work with same updated image , not given template
	cvtColor(newImage   ,grayROI   ,CV_BGR2GRAY);

    vector<Point2f> corners;  

	///* find new features set, when current matches are lower then minimum */
	//// ..meaning of aquiring new target..
  ///  if ( (trackedFeatures.size() < min_features * mid_level_percent ) )
	//	//if (FOUND_SOME_MOVEMENT == external_state)
	//	
	//	if ( (FOUND_GOOD_TARGET == system_state) || (newTargetSituation == true) )
	//	{
	//		goodFeaturesToTrack(grayTarget,corners,num_of_maxCornersFeatures,0.01,10);	//  int maxCorners, double qualityLevel, double minDistance,
	//		cout << "(re-)found " << corners.size() << " features\n";
	//		// set corners -> trackedFeatures 
	//		for (int i = 0; i < corners.size(); ++i) {
	//			trackedFeatures.push_back(corners[i]);
	//		}

	//		TrackPercent	= 100;
	//	}
	//	
	//else  // loss of features during tracking

	if ( (trackedFeatures.size() < min_features * mid_level_percent ) )
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

		if (current_trackingROI.x < 0) current_trackingROI.x = 0;
		if (current_trackingROI.y < 0) current_trackingROI.y = 0;
		if (current_trackingROI.x + current_trackingROI.width > prevGrayROI.size().width) 
			current_trackingROI.width -= 10;
		if (current_trackingROI.y + current_trackingROI.height > prevGrayROI.size().height) 
			current_trackingROI.height -= 10;

		
		prevGrayROI(current_trackingROI).copyTo(tmpIm);
		goodFeaturesToTrack(tmpIm, corners, num_of_maxCornersFeatures,0.01,10);	//  int maxCorners, double qualityLevel, double minDistance,
		//cout << "(re-)found " << corners.size() << " features\n";
	 
		// set corners -> trackedFeatures 
		trackedFeatures.clear();
		for (int i = 0; i < corners.size(); ++i) {
			/* store feature points in full image coordinates */
			corners[i].x = corners[i].x + current_trackingROI.x;
			corners[i].y = corners[i].y + current_trackingROI.y;
			trackedFeatures.push_back(corners[i]);
		}

        vector<uchar> status; vector<float> errors;
		// new input is trackedFeatures ->
		// new output is corners
		// status of 1 means correspondence found. 0 otherwise.
        //  calcOpticalFlowPyrLK(prevGrayROI,grayROI,trackedFeatures,corners,status,errors,Size(10,10));	// corners are 'InOut' array
        ///   calcOpticalFlowPyrLK(prevGrayROI,grayROI,trackedFeatures,corners,status,errors,Size(20,10));	// corners are 'InOut' array

		calcOpticalFlowPyrLK(prevGrayROI,grayROI,trackedFeatures,corners,status,errors,Size(20,10), 3,
			TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),0, 0.001);	//0.1 // corners are 'InOut' array

		/*////////// display the found feature points from	Prev (prevGrayROI, trackedFeatures)  to 
															Current (grayROI, corners)				 /////////*/
		Mat copyPrev; 
		///copyPrev	= prevGrayROI.clone();
		cvtColor(prevGrayROI.clone(),copyPrev,CV_GRAY2BGR);
		int r	= 2;	//3
		for( int i = 0; i < trackedFeatures.size(); i++ )
		{ 
			circle( copyPrev, trackedFeatures[i], r, 
				///Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); 
					Scalar(10, 100, 255), -1, 8, 0 ); 
		}			
		Mat copyCurrent;
		cvtColor(grayROI.clone(),copyCurrent,CV_GRAY2BGR);
		Mat copyCurrent2=copyCurrent.clone() ;
		Mat copyCurrent3=copyCurrent.clone() ;
		//int r	= 2;//3
		for( int i = 0; i < corners.size(); i++ )
		{ 
			if (status[i])
				circle( copyCurrent, corners[i], r, 
					///Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); 
					Scalar(0, 255, 0), -1, 8, 0 ); 
			else
				circle( copyCurrent3, corners[i], r+2,  
					Scalar(0, 0, 255), -1, 8, 0 ); 
			circle( copyCurrent2, corners[i], r, 
				///Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); 
				Scalar(10, 100, 255), -1, 8, 0 ); 
		}	

		///imshow("original copy ROI", copy);	


		IplImage	*im_mat1 = cvCloneImage(&(IplImage)copyPrev),
					*im_mat2 = cvCloneImage(&(IplImage)copyCurrent),
					*im_mat3 = cvCloneImage(&(IplImage)copyCurrent2),
					*im_mat4 = cvCloneImage(&(IplImage)copyCurrent3);
		//cvShowManyImages("images prev & current" , 4 , im_mat1, im_mat2, im_mat1, im_mat2 );
		cvShowManyImages("images prev & current " , 4 , im_mat1, im_mat2 , im_mat3, im_mat4 );

		if  (1==2)   // TODO: print only for 1st debugging
		cout  << "trackedFeatures size " << trackedFeatures.size() 
			<< " corners size " << corners.size() << " status size " 
			<< status.size() << " status non zeroes "<< countNonZero(status) <<"\n"; 

		///cvWaitKey(); ///

		/////////////////////////////////
		/*  just trial additional code 
		// Holds the colormap version of the image:
		Mat cm_img0;
		// Apply the colormap:
		applyColorMap(copy, cm_img0, COLORMAP_JET);
		// Show the result:
		imshow("cm_img0", cm_img0);*/
		/////////////////////////////////

		/// Create Trackbars
		////cout << TrackbarName << "Alpha x %d" << alphaSlider_max ;

		///createTrackbar( TrackbarName, "original copy ROI",	&alphaSlider, num_of_maxCornersFeatures, /*on_trackbar*/NULL );
		///createTrackbar( TrackbarName2, "original copy ROI", &alphaSlider2, alphaSlider_max, /*on_trackbar*/NULL );


		alphaSlider = countNonZero(status);
		alphaSlider2 = ((double)countNonZero(status))/ ((double)status.size()) * 100.0;
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

				//TODO: consier put this before this 'if'
			trackedFeatures.clear();
			for (int i = 0; i < status.size(); ++i) {
				if(status[i]) {
					trackedFeatures.push_back(corners[i]);
				}
			}
			/* calc center of points, and trackErr.x (bearing)  */
			///Rect newFeaturesRect = 
			Moments		m; 

			m					= moments(trackedFeatures,false);				// points moment 
			Point MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
			current_trackingROI	= OriginalTargetROI;// boundingRect( trackedFeatures ); 	
			int addedSpace = 2;
			current_trackingROI.x -= addedSpace; //TODO: change to *1.05 as 5% increase.. and check for staying in image limits
			current_trackingROI.y -= addedSpace;
			current_trackingROI.width  += addedSpace;
			current_trackingROI.height += addedSpace;

			TrkErrX_Readings[TrkErrX_readIndex] = MassCenter.x - prevGrayROI.size().width/2.0 ;
			TrkErrX_Avg = 0;
			for (int i=0; i< Nreads; i++)
				TrkErrX_Avg += TrkErrX_Readings[i];
			TrkErrX_Avg = TrkErrX_Avg / Nreads;
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
