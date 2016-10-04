//// 3D stero robot (RoboDog)
/*
	history can be vied in the GitHub site repository.
	written app : by Ran , year 2016
*/

/*********  main application constants object  ********/
#include "Headers\StereoRobotApp.hpp"
StereoRobotApp		myCApp;

/*********  thread object  ********/
#include ".\Headers\Depth_and_Disparity.hpp"		//set the disparity object (variables and functions)
Depth_and_Disparity localDisp;
/**********************************/

#include ".\Headers\frameFunctions.h"		// general definitions and functions. that's why it is first to include. 
 
StereoRobotApp::SYSTEM_STATUS	system_state = StereoRobotApp::INITIALIZING ;	//TODO: put under the main app OBJ


#include ".\Headers\BackgroundSub.hpp"
#include ".\OdroidC1_handlers\RobotController.h"

#include ".\Headers\stereo_calib.h" 
#include ".\Headers\FeatureTracker.hpp"
#include ".\Headers\ImagesSourceHandler.h"
/*********  GUI object  ********/
#include ".\Headers\myGUI_handler.h"
myGUI_handler myGUI;				// class for displaying the images
/*  ***************************  */
 

#ifdef COMPILING_ON_ROBOT
////#include "OdroidC1_handlers/RobotController.h"
#include <unistd.h>
#include "pwm.h"
#endif

#include <mutex>
#include <atomic> 
#include <memory>
#include <thread>

//#include "imporeted_raw_code_examples/watershed_from_OpencvSamples.cpp"
//#include "imporeted_raw_code_examples/backSubsExample.cpp"
//#include "imporeted_raw_code_examples/grabcut_from_OpencvSamples.cpp"//
#include "imporeted_raw_code_examples/imageSegmentation.cpp"//
///#include "imporeted_raw_code_examples/ffilldemo.cpp"//
#include "imporeted_raw_code_examples/kalman.cpp"


////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////					  main	   			  //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////// 

int main(int argc, char** argv) 
{
	/*******************************************************************/
	bool	raw_sensors_run = true;		// in order to test cardinal algorithms (for pseudo-sensors)
	/*******************************************************************/

 	/* variables */
	Mat left_im_color ,
		right_im_color;
	Mat left_im_gray ,
		right_im_gray;


	RobotController hardwareController ;


	myCApp.op_flags.show_stereo=true;	// initialize and conduct stereo algo imidiatly when running.

	Mat		target_candidate_features;

	bool request_water_shed = false;

	const int		loop_delay = 33 ; //[ms]	// need to fit the file recording value
	char	user_pressing=0;	// just optional.

	int relative_counter =0;
	
	/*  initiating images capturing  */
	ImagesSourceHandler myStereoCams; // thread for images capturing
	/*  ***************************  */

	/*  initiating createBackgroundSubtractorMOG2   */
	BackSubs	localBackSubs ;
	/*  *****************************************   */
	/*  initiating target   */
	Target	first_target ;
	/*  *****************************************   */
	const int target_lost_timeout_counter  = 2* loop_delay ; // [~sec]//counter to simulate delay of about 2 sec. (depend on loop inner delay)
	int		  target_lost_time_counter = 0 ;   // stopper to timeout

	Tracker		tracker;
	Rect		BckgndSubROI;
	Rect		TrackingROI;

	Mat			lastDepthImg,
				depthAvg;
	int			depthAvgNdx = 0;

	bool got_1st_stable_bkgnd = false;

	Depth_and_Disparity::rectification_outputs disperity_struct;
	Mat		disp_temporary		= Mat();
	Mat		modified_disperity_mat;
	Scalar	avg_disperity_S;
	double	avg_disperity;
	double	avg_depth_of_ROI	= 0 ;
	double	last_depth_of_ROI	= 0 ;

	Point	movementMassCenter, corected_MassCenter;
	Mat		current_mask1,
			current_mask2;
	Mat		bgnd;

	bool	gotNewDispImageToWorkWith = false;

	myMatQueue<Mat>		matQueue;
	myMatQueue<double>	doubleQueue;					
	
	Mat featTrackMask1, 
		featTrackMask2,
		featTrackMask;		//M1 & M2 -> M

	Mat					tmpROI;
	Target::TargetState tmpTargStat;

	/* end of variables */
	
	////////////// initializations ///////////
	
	int w		= myStereoCams.GetRes().width,	
		h		= myStereoCams.GetRes().height, 
		waitSec = 5;

	localBackSubs.show_forgnd_and_bgnd_init(30); //vidL//with Left cam  

		/* clear points that are out of my desired ROI (center of image) */
		//TODO:make 20 h , 30 w /// sizes are for after resize
		//CV_EXPORTS_W void rectangle(InputOutputArray img__, Point pt1, Point pt2,
		//						const Scalar& color, int thickness = 1,
		//						int lineType = LINE_8, int shift = 0); 
	Point	TopLeft(myCApp.frame_boundary, myCApp.frame_boundary); 
	Point	LowRight(w - myCApp.frame_boundary , h - myCApp.frame_boundary);
	TrackingROI = Rect(TopLeft, LowRight ); 

	//	ROI for background substraction is narrower then the one for the tracker
	TopLeft		 = Point(myCApp.frame_boundary_W_init		, myCApp.frame_boundary	); 
	LowRight	 = Point(w - myCApp.frame_boundary_W_init	, h - myCApp.frame_boundary);
	BckgndSubROI = Rect(TopLeft, LowRight ); 

	////////////// end of initializations ///////////

	//	if (!raw_sensors_run), first_target

	myWaterShed WSHD;
	Mat distancesMask;


	///mainKF();
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////				main loop	   			  //////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////// 
	
	while (1)		
	{

#ifndef COMPILING_ON_ROBOT
		if (myCApp.op_flags.make_stereo_calibration)
		{		
			argc = 6;
			argv[1] = "-w";  argv[2] = "8";
			argv[3] = "-h";  argv[4] = "6";
			argv[5] = "../run_inputs/stereo_calibration_images/stereo_calib.xml";
			do_stereo_calib(argc, argv);
			myCApp.op_flags.make_stereo_calibration	=	false;
		}
#endif

#ifndef COMPILING_ON_ROBOT
		if(myCApp.op_flags.show_stereo)
#endif
		{
			////////////// capture (color) images ///////////
			if ( !myStereoCams.GetFrames( myGUI.plotImages[0], myGUI.plotImages[1] ) )	
				if(relative_counter==0)
					continue;
				else
					break;
			
			relative_counter++;

			right_im_color  = myGUI.plotImages[0].clone();   
			left_im_color   = myGUI.plotImages[1].clone();   

			if (myStereoCams.GetUserRepeatFlag())
			{
				// if testing recorded files - don't continue to read image frames in background. 
				// until next deterministic loop request. (specially if stopping for debugging).
				myStereoCams.ToggleDisableFramesCapture();
			}

			////////////// end of capture images ///////////

			/// show raw images first
			Mat tmpLeft = left_im_color.clone();
			long tmpLong = myStereoCams.GetFrameCycleCounter();
			myGUI.add_counterFrame(tmpLeft , &tmpLong ) ;
			imshow(myGUI.plotWindowsNames[0],	right_im_color );
			imshow(myGUI.plotWindowsNames[1],	tmpLeft );
			///

			switch (system_state) 
			{
				case StereoRobotApp::INITIALIZING:
/* *********** */	//localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
					//if (localBackSubs.BgSubt_Status == BackSubs::STANDING_BY)
						system_state = StereoRobotApp::STANDBY;
					///WSHD.init_mask_by_input(left_im_color) ;
					break;

				case StereoRobotApp::STANDBY:
/* *********** */	//localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
					//if (localBackSubs.BgSubt_Status == BackSubs::FOUND_MOVEMENT)
						system_state = StereoRobotApp::FOUND_SOME_MOVEMENT;
					break;

				case StereoRobotApp::FOUND_SOME_MOVEMENT:
					/************************bgSubt************************/
/* *********** */	//localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
					/************************disp************************/
/* *********** */	localDisp.calc_disperity(3, left_im_color, right_im_color, &disp_temporary , &last_depth_of_ROI );  //2 , when 1 above
							 
					matQueue.populateNextElementInArray(disp_temporary); 
					matQueue.getSumElement(&disp_temporary);

					doubleQueue.populateNextElementInArray(last_depth_of_ROI);
					doubleQueue.getAvgElement(&avg_depth_of_ROI);
					 
					/********************feat.tracker****************************/

					int depth;		// rounded to [cm]
					//featTrackMask1 = localBackSubs.get_foreground_mat();
					///localDisp.get_filtered_disparity(featTrackMask2, &depth);
					featTrackMask2	=	disp_temporary.clone();
					depth			=	avg_depth_of_ROI;
					//featTrackMask1.copyTo (featTrackMask, featTrackMask2 ) ;	// output is featTrackMask	
					///featTrackMask2.copyTo (featTrackMask);
					featTrackMask = featTrackMask2.clone();
					threshold (featTrackMask , featTrackMask ,	1 ,	255,THRESH_BINARY);		// take all that is not zero THRESH_TOZERO

					first_target.calc_target_mask_properties(featTrackMask) ;
					if (!first_target.check_target_mask_properties_option1())
					{

						if (myStereoCams.GetUserRepeatFlag())
						{
							// if testing recorded files - don't continue to read image frames in background. 
							// until next deterministic loop request. (specially if stopping for debugging).
							///	waitKey(0*loop_delay);
							myStereoCams.ToggleDisableFramesCapture();
							waitKey( loop_delay );
						}
						continue;
					}

					cv::cvtColor(left_im_color , left_im_gray  , CV_BGR2GRAY);
					cv::cvtColor(right_im_color, right_im_gray , CV_BGR2GRAY);
					
					///	mainImSeg(left_im_color);	// need to use the markers with depth ROI anti-mask as sure background.
													// or use the inside use of distanceTransform to better to find the bigger object.

					//// tmpROI is MASK in potential area of new target. 
					tmpROI							=  Mat::zeros( left_im_gray.size() , left_im_gray.type() );
					first_target.potential_target	= tmpROI.clone();

					tmpROI( first_target.target_mask_prop.boundRect ) = 255;
					// potential target is Trimmed area acording to ROI (from Depth or BgSubt)					
					left_im_gray.copyTo(first_target.potential_target , tmpROI); 

					bool trackerNotOff = false;
					if ( tracker.Tracker_State != Tracker::TRACKER_OFF)
						trackerNotOff = true;

/* *********** */	tracker.processImage(left_im_gray, first_target.potential_target, first_target.target_mask_prop.boundRect );// localBackSubs.get_foreground_boundRect() );

					if ( (trackerNotOff) && (tracker.Tracker_State == Tracker::TRACKER_OFF) )	//back from advanced mode to OFF
					{
						system_state	=	 StereoRobotApp::INITIALIZING ;
						//TODO: close potential_target window
						break ;
					}

					circle(first_target.potential_target, corected_MassCenter, 4, Scalar(255, 155, 55 ), -1, 8, 0);
					circle(first_target.potential_target, tracker.MassCenter,  4, Scalar(0  , 255, 255), -1, 4, 0);	//yellow point.

					imshow(myGUI.plotWindowsNames[4], first_target.potential_target);//8
					////////

					// check condition for GOOD_TRACKING.   
					tmpTargStat = tracker.trackedTarget.check_target_mask_properties() ;
					//if (tmpTargStat==Target::Target_present)
					//	//system_state = TRACKING_GOOD_QUALITY_TARGET ;
					//	system_state = StereoRobotApp::FOUND_GOOD_TARGET ;

				/*	if (tracker.Tracker_State == Tracker::TRACKER_TRACKING)
						system_state = StereoRobotApp::FOUND_GOOD_TARGET;
					*/

					/************************************************/
					
					myGUI.add_distance_to_disparityIM(avg_depth_of_ROI, disp_temporary);

					imshow ( myGUI.plotWindowsNames[3], disp_temporary );	 
					 

					break;

				case StereoRobotApp::FOUND_GOOD_TARGET:
	
					corected_MassCenter = Point(movementMassCenter.x + BckgndSubROI.x,  movementMassCenter.y + BckgndSubROI.y); 

					current_mask1 = localBackSubs.get_foreground_mat() ; 
					current_mask1.copyTo (current_mask2, lastDepthImg ) ;		//2nd mask , composing together

					left_im_color.copyTo ( first_target.potential_target, current_mask2 ) ;	//foreground is a region mask 
					//potential_target.copyTo ( potential_target, lastDepthImg ) ;					//lastDepthImg is a 2nd region mask 
					 

/* *********** */	tracker.processImage(left_im_gray, first_target.potential_target, localBackSubs.get_foreground_boundRect() );

					circle(first_target.potential_target, corected_MassCenter, 4, Scalar(255, 155, 55), -1, 8, 0);
					circle(first_target.potential_target, tracker.MassCenter, 4, Scalar(0, 25, 25), -1, 4, 0);	//yellow point.

					imshow(myGUI.plotWindowsNames[4], first_target.potential_target);//8
					 
					// track forward

					// TODO : add sliding bar to animate Thrust.
					if (system_state == StereoRobotApp::FOUND_GOOD_TARGET)
						if ((avg_depth_of_ROI>15) && (avg_depth_of_ROI<999)) // 5 as minimum depth to go to
						{
							 //thrust_percent = 50; //make static somwhere
							 //angle = -45 /57.3;		 //-45deg
							//double thrust_per = (avg_depth_of_ROI-Dmin)/(Dmax-Dmin);
							double thrust_per = (avg_depth_of_ROI-11)/(190-11)*100.0*5.0;
							hardwareController.Forward(thrust_per,  0, 0);
						}
						else 
						{
							hardwareController.Stop();
						}
					else
					{
						hardwareController.Stop();
					}


					////////////// added graphics section /////////// 

					if (myCApp.op_flags.draw_middle_x)
					{			
						Point2f targetCenter ;
						// TODO: test source of track errors, from BackgroundSubs, or Tracker
						///	add_Cross_to_Image(tracker.TrkErrX  ,  left_im_color.size().height/2  , 
						targetCenter	=	tracker.MassCenter;
						if (system_state >= StereoRobotApp::FOUND_GOOD_TARGET) 
							myGUI.add_Cross_to_Image(targetCenter.x  ,  targetCenter.y  , 
								false, system_state , left_im_color); //	changes input image. but it is last in the flow cycle.
					}

					break;
			}											
			
			// move to broadcast from outside. // separate the graphic layer to only when track is on.
			//									otherwise just draw raw images.

			tmpLong = myStereoCams.GetFrameCycleCounter();
			myGUI.add_counterFrame(left_im_color , &tmpLong ) ;
			imshow(myGUI.plotWindowsNames[0],	myGUI.plotImages[0] );
			imshow(myGUI.plotWindowsNames[1],	left_im_color);
			 
			////////////* end of graphics section *///////////////
		}

		/////////////////////////////////////////////////////////////////////
		///////////////////////user response handling////////////////////////
		/////////////////////////////////////////////////////////////////////
/*
		if (myStereoCams.GetUserRepeatFlag())
		{
			myStereoCams.ToggleDisableFramesCapture();
		}*/
		int c = waitKey(loop_delay);

		if (myStereoCams.GetUserRepeatFlag())
		{
			waitKey(0*loop_delay);
			myStereoCams.ToggleDisableFramesCapture();
			waitKey( loop_delay );
		}

		if (c==(int)'w')
			request_water_shed = true;


		double thrust_percent, angle;
		switch (c) {
		case StereoRobotApp::UNIX_KEY_UP:
			 thrust_percent = 50; //make static somwhere
			 angle = 0;			
			hardwareController.Forward(thrust_percent, angle, 0);
			break;
		case StereoRobotApp::UNIX_KEY_DOWN:
			hardwareController.Stop();
			break; 
		case StereoRobotApp::UNIX_KEY_LEFT:
			 thrust_percent = 50; //make static somwhere
			 angle = -45 /57.3;		 //-45deg
			hardwareController.Forward(thrust_percent, angle, 0.9);
			break;
		case StereoRobotApp::UNIX_KEY_RIGHT:
			 thrust_percent = 50; //make static somwhere
			 angle = 45 /57.3;		
			hardwareController.Forward(thrust_percent, angle, 0.9);
			break;
		}

		if (c==27)
			break; 
	}
	
	return 0;

}