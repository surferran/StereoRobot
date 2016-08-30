//// 3D stero robot (RoboDog)
/*
	history can be vied in the GitHub site repository.
	written app : by Ran , year 2016
*/
#define RUN_ON_LAPTOP__MONO true    

 /* my  constants and parameters */

// bounds in percent from image size
#define MIN_MOVED_AREA_in_image 33//33//.0//23
#define MAX_MOVED_AREA_in_image 95.0
#define NUM_OF_PIXELS_IN_FRAME	(640.0*480.0)
#define MIN_CURVE_AREA			MIN_MOVED_AREA_in_image/100*NUM_OF_PIXELS_IN_FRAME
#define MAX_CURVE_AREA			MAX_MOVED_AREA_in_image/100*NUM_OF_PIXELS_IN_FRAME
#define SHOW_MOVING_CONTOURS		true//true
#define SHOW_MOVING_RECTANGLES		false//true
#define SHOW_MOVING_BIG_CONTOURS	true
#define SHOW_MOVING_BIG_RECTANGLES	false//true

// below or above those limits it will be treated as noise. (or too close movement to camera)
#include "stereo_functions.hpp"

#include "frameFunctions.h"		// general definitions and functions. that's why it is first to include.
#include "working_consts.h"		// my added definitions, constants

StereoCams		thisStereo;				// global 
SYSTEM_STATUS	system_state = INITIALIZING ;
Operation_flags	op_flags; //global

#include "BackgroundSub.hpp"

//#include "GUIFunctions.h"

// also : https://en.wikipedia.org/wiki/Image_moment#Examples_2
///#include "camshiftdemo.cpp"

///#include "./SBM_Sample.cpp"		// externals/

#include "stereo_calib.h" 

#include "FeatureTracker.hpp"

#ifdef COMPILING_ON_ROBOT
////#include "OdroidC1_handlers/RobotController.h"
#include <unistd.h>

#include <thread>
#include <memory>
#include "pwm.h"
#endif

///
#include "ImagesSourceHandler.h"
#include "myGUI_handler.h"


//#include "showManyImages.cpp"
void cvShowManyImages(char* title, int nArgs, ...) ;
///void processImage(Mat& imgTarget, Mat& imgROI , SYSTEM_STATUS external_state); //#include 'myTracker.cpp'
////when need to eliminate the consule that is opened in parallel 
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

int findBiggestContour(vector<vector<Point> > contours){
	int indexOfBiggestContour = -1;
	int sizeOfBiggestContour = 0;
	for (int i = 0; i < contours.size(); i++){
		if(contours[i].size() > sizeOfBiggestContour){
			sizeOfBiggestContour = contours[i].size();
			indexOfBiggestContour = i;
		}
	}
	return indexOfBiggestContour;
}
void makeContours(Mat aBw){

	vector<vector<Point>>	contours;
	vector<vector<int>>		hullI;
	vector<vector<Point>>	hullP;
	vector<vector<Vec4i>>	defects;	
	int						cIdx;
	Rect					bRect;

	findContours(aBw, contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	hullI	=vector<vector<int> >	(contours.size());
	hullP	=vector<vector<Point> >	(contours.size());
	defects	=vector<vector<Vec4i> > (contours.size());
	cIdx	=findBiggestContour( contours);
	if( cIdx!=-1){ 
		bRect=boundingRect(Mat( contours[ cIdx]));		
		convexHull(Mat( contours[ cIdx]), hullP[ cIdx],false,true);
		convexHull(Mat( contours[ cIdx]), hullI[ cIdx],false,false);
		approxPolyDP( Mat( hullP[ cIdx]),  hullP[ cIdx], 18, true );
		///if( contours[ cIdx].size()>3 )
		{
		///	convexityDefects( contours[ cIdx], hullI[ cIdx], defects[ cIdx]);
			 ///eleminateDefects(m);
		} 
		Scalar color;

		color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours(aBw, contours, cIdx , color );

		color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours(aBw, hullP   , cIdx , color );

		rectangle(aBw,
			bRect.tl(), bRect.br(),
			Scalar(128,220,220) ,	2, 8, 0);

		imshow("hull results", aBw);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////					  main	   			  //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) 
{
 

	/* variables */

	thisStereo.input_source = LIVE_CAM;
		

	op_flags.show_stereo=true;	// initialize and conduct stereo algo imidiatly when running.

///	string	base_out_file_path	= "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/out";
	string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".*";	
	int		stream_frame_index	= 0;
	/*char	rec_file_name[150]  = "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/in/VID_3D_scenario/output_1.avi";
	VideoCapture vid			= VideoCapture(rec_file_name);	*/
	Mat		target_candidate_features;
	///Mat		tracked_target_image;

	bool first_setup = true;

	const int		loop_delay = 33 ; //[ms]	// need to fit the file recording value
	char	user_pressing=0;	// just optional.

	int relative_counter =0;
	
	/*  initiating images capturing  */
	ImagesSourceHandler myStereoCams; // thread for images capturing
	/*  ***************************  */
	/*  initiating GUI setup*/
	myGUI_handler myGUI; // thread for images displaying
	/*  ***************************  */

	const int target_lost_timeout_counter  = 2* loop_delay ; // [~sec]//counter to simulate delay of about 2 sec. (depend on loop inner delay)
	int		  target_lost_time_counter = 0 ;   // stopper to timeout


	//plotWindowsNames[0] = "win1 - right stereo image";
	//plotWindowsNames[1] = "win2 - left stereo image"; 
	//	// TODO: added frame of copmosed stiched image ? can take relative very long process.
	//		// anyway can add as ref. with other low-prio thred
	//	//  it is actually part of the disparity process.
	//plotWindowsNames[2] = "win3 - calculated disparity";
	//plotWindowsNames[3] = "win4 - background substruction output";
	//plotWindowsNames[4] = "win5 - tracked object";

	/*  initiating createBackgroundSubtractorMOG2   */
	BackSubs	localBackSubs ;
	/*  *****************************************   */

	Tracker		tracker;
	Rect		BckgndSubROI;
	Rect		TrackingROI;

	bool got_1st_stable_bkgnd = false;

	/* end of variables */

	////////////// 1st entrance only ///////////
	if (first_setup) {
		first_setup = false;

		int w=320,	h=240 , waitSec = 5;

		localBackSubs.show_forgnd_and_bgnd_init(30); //vidL//with Left cam  


														/* clear points that are out of my desired ROI (center of image) */
														//TODO:make 20 h , 30 w /// sizes are for after resize
														//CV_EXPORTS_W void rectangle(InputOutputArray img, Point pt1, Point pt2,
														//	const Scalar& color, int thickness = 1,
														//	int lineType = LINE_8, int shift = 0); 
		Point	TopLeft(frame_boundary, frame_boundary); 
		Point	LowRight(w - frame_boundary , h - frame_boundary);
		TrackingROI = Rect(TopLeft, LowRight ); 

		//	ROI for background substraction is narrower then the one for the tracker
		TopLeft		 = Point(frame_boundary_W_init		, frame_boundary	); 
		LowRight	 = Point(w - frame_boundary_W_init	, h - frame_boundary);
		BckgndSubROI = Rect(TopLeft, LowRight ); 
	}
	////////////// end of 1st entrance only ///////////

	// prepare display windows locations
	
	//drawMatches
	
	//TODO:  show initial target , and then current updated last target. if no target then show non or close win.
	//   (default image shoult be zeroes(res.x, res.y) )
	//		allow 1 or 2 frames to be with no featres. set a GapCounter.

	// show normal status for tracker status.

	if (!RUN_ON_LAPTOP__MONO)
		do_stereo_disp_init();


	while (1)		
	{
		if (op_flags.make_stereo_calibration)
		{		
			argc = 6;
			argv[1] = "-w";  argv[2] = "8";
			argv[3] = "-h";  argv[4] = "6";
			argv[5] = "../run_inputs/stereo_calibration_images/stereo_calib.xml";
			do_stereo_calib(argc, argv);
			op_flags.make_stereo_calibration	=	false;
		}

		if(op_flags.show_stereo)
		{
			////////////// capture images ///////////
			myStereoCams.GetFrames(myGUI.plotImages[0],myGUI.plotImages[1]);
			if ((myGUI.plotImages[0].empty()) )
				continue;
			if ((myGUI.plotImages[1].empty()) )
				continue;
			
			relative_counter++;
			if (relative_counter==1)
				continue;			//just for getting debug point..
			
			////////////// end of capture images ///////////
			 
			Mat left_cam = myGUI.plotImages[1].clone();   

			Point movementMassCenter, corected_MassCenter;
			// condition by STANDBY, otherwise - only the tracker is in the loop 
			if ( system_state < FOUND_GOOD_TARGET )
			// will change system_state only when (system_state <= FOUND_SOME_MOVEMENT )
			{
				localBackSubs.find_forgnd( left_cam(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
				corected_MassCenter = Point(movementMassCenter.x + BckgndSubROI.x,  movementMassCenter.y + BckgndSubROI.y);
				////actually not needed .. makeContours(localBackSubs.get_foreground_mat()); 
			}
			////////////* get DEPTH by Stereo *///////////////
			if (!RUN_ON_LAPTOP__MONO){
				// calc disparity every 2 frame
				if (relative_counter>1) //10  
				{
					cv::cvtColor(myGUI.plotImages[0+1], myGUI.plotImages[0+1], CV_BGR2GRAY);
					cv::cvtColor(myGUI.plotImages[0*1], myGUI.plotImages[0*1], CV_BGR2GRAY);

					do_stereo_disp(myGUI.plotImages[0+1],myGUI.plotImages[1*0], myGUI.plotImages[2]);  // plotImages[2] is the disparity output
					///disp relevant disperity values. 
					// for image blobs or average areas. use superpixel segmentation??
					// get disp average for the target feature poitns area.

					//main_SBM(plotImages[0+1],plotImages[1*0], plotImages[2]); 
					imshow(myGUI.plotWindowsNames[2],  myGUI.plotImages[2]);
					relative_counter	=	0;
				}
			}
			////////////* get DEPTH by stereo *///////////////


			////////////////////////////////////////////////
			//			tracking part (by 'goodFeatures')
			if ( system_state == FOUND_GOOD_TARGET )
			{	// want to init and lock the tracker
				// get the feature points of the target from the BackgroundSubs ROI
				///Mat candidate_features;
				Mat tracked_target_image;
				Rect corrected_ROI = Rect(	BckgndSubROI.x  + localBackSubs.get_foreground_boundRect().x ,
											BckgndSubROI.y  + localBackSubs.get_foreground_boundRect().y ,
											localBackSubs.get_foreground_boundRect().width ,
											localBackSubs.get_foreground_boundRect().height ) ;

				//target_candidate_features = left_cam.clone(); 
				//target_candidate_features = Scalar(0,0,0);
				//target_candidate_features(BckgndSubROI) = localBackSubs.get_foreground_mat();//*left_cam ;	//element-wise multiplication

				//cvtColor( target_candidate_features , candidate_features , CV_GRAY2BGR);

				//candidate_Features = candidate_Features.mul(left_cam(BckgndSubROI)  ) ; 
				//candidate_Features = left_cam(BckgndSubROI)  . mul(candidate_Features);
				///left_cam(BckgndSubROI).copyTo(tracked_target_image, candidate_features);
				left_cam(corrected_ROI).copyTo(tracked_target_image);
				imshow("tracked Target start", tracked_target_image) ; // show 4 debug  only

				tracker.setNewTarget(corrected_ROI, tracked_target_image, TrackingROI);

			//	Mat candidate_Features2 = candidate_Features * left_cam(BckgndSubROI) ;	//element-wise multiplication
			//	imshow("Target candidates2", candidate_Features2) ; // show 4 debug  only
					// TODO: check minimum number of quality feature points of that target
					//	otherwise it is low quality tracking	
					//tracker.processImage(candidate_Features2,  left_cam (TrackingROI) , system_state);  
				//find 
			}

			if (system_state >= FOUND_GOOD_TARGET)
			{

		///		if (target_lost_time_counter == 0)   // otherwise wait for ..lost.. (should be a time-window of recapture target)
				//if ( system_state == FOUND_SOME_MOVEMENT )
				//{
				//	// get the feature points of the target from the BackgroundSubs ROI
				//	Mat candidate_Features = localBackSubs.get_foreground_mat();//*left_cam ;	//element-wise multiplication
				//	imshow("Target candidates", candidate_Features) ; // show 4 debug  only
				//	// TODO: check minimum number of quality feature points of that target
				//	//	otherwise it is low quality tracking	
				//	tracker.processImage(candidate_Features,  left_cam (TrackingROI) , system_state);  

				//}
				//else		  
					////////////////////////////////////////////////
					//			make tracking of the 'goodFeatures'
					//			from previous frame to the new one		
				tracker.processImage(left_cam.clone()  ,system_state);  

				Point2f targetCenter ;

				if ( system_state == FOUND_GOOD_TARGET )
					targetCenter = localBackSubs.get_foreground_center() ;
				else
					targetCenter = Point(tracker.TrkErrX_Avg + left_cam.size().width/2 ,  left_cam.size().height/2  );

				if (tracker.TrackPercent > 65)
					system_state = TRACKING_GOOD_QUALITY_TARGET;
				else 
				if (tracker.TrackPercent > 20)
					system_state = TRACKING_LOW_QUALITY_TARGET;
				else
					if ( ( (tracker.TrackPercent > 5) && (target_lost_time_counter < target_lost_timeout_counter) ) //95
						 && (target_lost_time_counter >= 0) )
					{	
						system_state = TARGET_IS_LOST;
						target_lost_time_counter ++;
					}
					else
					{
						system_state	= INITIALIZING;
						target_lost_time_counter		=	0;
					}
				//// check direction change from the previous tracked poits center. mark an arrow .
				//////////////////////////////////////////////////
				////	calculate translation from previous to current.
				////	display modified current 
		//		Mat invTrans = tracker.rigidTransform.inv(DECOMP_SVD);
		//		Mat orig_warped;
		//		warpAffine(left_cam,orig_warped,invTrans.rowRange(0,2),Size());
		//		imshow("stabilized",orig_warped);
				
				

				////////////// added graphics section ///////////
				left_cam	= myGUI.plotImages[1].clone();   // for some additional display layer

				if (op_flags.draw_middle_x)
				{
					//on the right image
					//add_Cross_to_Image(left_cam.size[1]/2, left_cam.size[0]/2, false, system_state , left_cam); // 120h,160w , with no coor. label

					// TODO: test source of track errors, from BackgroundSubs, or Tracker
				///	add_Cross_to_Image(tracker.TrkErrX  ,  left_cam.size().height/2  , 
					myGUI.add_Cross_to_Image(targetCenter.x  ,  targetCenter.y  , 
											false, system_state , left_cam); // 120h,160w , with no coor. label
					
				}
				
				////////////* end of graphics section *///////////////
			}
			// move to broadcast from outside. // separate the graphic layer to only when track is on.
			//									otherwise just draw raw images.
			imshow(myGUI.plotWindowsNames[0],	myGUI.plotImages[0] );
			imshow(myGUI.plotWindowsNames[1],	left_cam);
			 
		}

		char c = (char)waitKey(loop_delay);
		if (c==27)
			break;
		//if (!check_user_input(&loop_delay, &user_pressing))
			//break;
	}
	
	return 0;

}