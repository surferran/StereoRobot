/*
 *  StereoRobotApp.cpp  
 */

#include "StereoRobotApp.hpp"

 /**************************************/

#include "myGUI_handler.h"
myGUI_handler		myGUI;

ImagesSourceHandler myStereoCams; // thread for images capturing

Target				tracked_target ;

Tracker				tracker; 

Depth_and_Disparity	localDisp;

BackSubs			localBackSubs ;

/**************************************/

/* class initiation */
StereoRobotApp::StereoRobotApp()
{ 
	working_FRAME_WIDTH		=	320 /1 ;// 640;// 160;
	working_FRAME_HIGHT		=	240 /1 ;// 480;// 120;

	frame_boundary_W_init	=	0;
	frame_boundary			=	0;

	minDepth_toGoTo			=	15;	//[cm]
	maxDepth_toGoTo			=	300;//[cm]

	op_flags.make_stereo_calibration	=	false;
	op_flags.calc_background_subs		=	false;
	op_flags.show_vid_source_selection	=	false;

	op_flags.show_stereo				=	false;
	op_flags.play_on					=	false;
	op_flags.save_disparity_img_to_file =	false; 

	op_flags.make_camshift				=	false;

	op_flags.reset_vid_file_location	=	false;
	op_flags.reset_identification		=	false;

	op_flags.show_img_hist				=	false;
	op_flags.proces_img_frame			=	false;
	op_flags.draw_middle_x				=	true ;

	initialUserFwdThrust_percent		=	50;	//[%]

};


StereoRobotApp::~StereoRobotApp(){};

/************************************************************/

void StereoRobotApp::appInitializations()
{
	op_flags.show_stereo	= true;	// initialize and conduct stereo algo imidiatly when running.

	user_pressing			= 0;	// just optional.
	got_1st_stable_bkgnd	= false;

	w		= myStereoCams.GetRes().width;
	h		= myStereoCams.GetRes().height;

	localBackSubs.show_forgnd_and_bgnd_init(working_FRAMES_FPS, false); 

	/* clear points that are out of my desired ROI (center of image) */
	//TODO:make 20 h , 30 w /// sizes are for after resize
	Point	TopLeft(frame_boundary, frame_boundary); 
	Point	LowRight(w - frame_boundary , h - frame_boundary);
	TrackingROI = Rect(TopLeft, LowRight ); 

	//	ROI for background substraction is narrower then the one for the tracker
	TopLeft		 = Point(frame_boundary_W_init		, frame_boundary	); 
	LowRight	 = Point(w - frame_boundary_W_init	, h - frame_boundary);
	BckgndSubROI = Rect(TopLeft, LowRight ); 

	userFwdThrust_percent	=	50;
}


void StereoRobotApp::check_for_calibration_option()
{
#ifndef COMPILING_ON_ROBOT
	if (op_flags.make_stereo_calibration)
	{		
		int argc; char* argv[6];
		argc = 6;
		argv[1] = "-w";  argv[2] = "8";
		argv[3] = "-h";  argv[4] = "6";
		argv[5] = "../run_inputs/stereo_calibration_images/stereo_calib.xml";
		do_stereo_calib(argc, argv);
		op_flags.make_stereo_calibration	=	false;
	}
#endif
}

void StereoRobotApp::appMainLoop()
{

	while (1)		
	{
		check_for_calibration_option();

#ifndef COMPILING_ON_ROBOT
		if(op_flags.show_stereo)
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
			cv::cvtColor(right_im_color, right_im_gray , CV_BGR2GRAY);	//
			cv::cvtColor(left_im_color , left_im_gray  , CV_BGR2GRAY);	// can include in GUI capture?

			/* if testing recorded files - don't continue to read image frames in background. 
			    until next deterministic loop request. (specially if stopping for debugging).*/
			if (myStereoCams.GetUserRepeatFlag())
			{
				myStereoCams.ToggleDisableFramesCapture();
			}

			////////////// end of capture images ///////////

			myGUI.show_raw_captures(left_im_color, right_im_color, myStereoCams.GetFrameCycleCounter(), system_state);

			/************************bgSubt************************/
			if ( (system_state == INITIALIZING) || (system_state == STANDBY) || (system_state == FOUND_SOME_MOVEMENT) )
			{
				localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
																								 //
				if (localBackSubs.BgSubt_Status == BackSubs::STANDING_BY)
					system_state = StereoRobotApp::STANDBY; 
				else 
					if (localBackSubs.BgSubt_Status == BackSubs::FOUND_MOVEMENT)
					{
						system_state	= StereoRobotApp::FOUND_SOME_MOVEMENT;
						featTrackMask_fromBgSubt	= localBackSubs.get_foreground_mat(); 
					}					
			}
			else
			{
				localBackSubs.show_forgnd_and_bgnd_init(working_FRAMES_FPS, false);
				myGUI.close_BgSubt_win();  // TODO: just add red frame, and when operative set to green
			}
			
			if ( (system_state == FOUND_SOME_MOVEMENT) || (system_state == TRACKING) )
			{
				/************************disp************************/
				localDisp.calc_disperity(3, left_im_gray, right_im_gray, &current_disparity , &last_min_depth_of_ROI ); // also filters

				matQueue.populateNextElementInArray(current_disparity); 
				matQueue.getSumElement(&sum_of_N_disparities);

				doubleQueue.populateNextElementInArray(last_min_depth_of_ROI);
				doubleQueue.getAvgElement(&avg_depth_of_ROI);

				featTrackMask_fromDisperity	=	sum_of_N_disparities.clone();		// sum of last 3 frames
					 	//TODO: drive should be around D closer. and around ROI.  in tracker - dont drag FP that are not in D ROI ( or vrery far..)
				threshold (featTrackMask_fromDisperity , featTrackMask_fromDisperity ,	1 ,	255,THRESH_BINARY);	// take all that is not zero 
				effective_depth_measurement		=	avg_depth_of_ROI;		// rounded to [cm]
			
				/********************feat.tracker****************************/
				  
				featTrackMask	= Mat::zeros(left_im_gray.size(), left_im_gray.type() ) ; 
				if (system_state == StereoRobotApp::FOUND_SOME_MOVEMENT)
					featTrackMask_fromBgSubt.copyTo (featTrackMask, featTrackMask_fromDisperity ) ;	// output is featTrackMask , by mask1 && mask2
				else
					featTrackMask = featTrackMask_fromDisperity.clone(); // TODO:TODO: need to combine with previous FP ROI

				//	mainImSeg(left_im_color);	// need to use the markers with depth ROI anti-mask as sure background.
				// or use the inside use of distanceTransform to better to find the bigger object.

				trackerNotOff = false;
				if ( tracker.Tracker_State != Tracker::TRACKER_OFF)
					trackerNotOff = true;

				tracked_target.set_target_mask_properties(featTrackMask);
				tracker.processImage(left_im_gray, &tracked_target) ;	//tracked_target is IN/OUT object

				if ( (trackerNotOff) && (tracker.Tracker_State == Tracker::TRACKER_OFF) )	//back from advanced mode to OFF
				{
					system_state	=	 StereoRobotApp::INITIALIZING ;
					localBackSubs.show_forgnd_and_bgnd_init(0,true);
					myGUI.close_Tracking_win();
				}
				if ( tracker.Tracker_State == Tracker::TRACKER_TRACKING )	//back from advanced mode to OFF
				{
					system_state	=	 StereoRobotApp::TRACKING ; 
					//continue;	
				}

				/******************** robot control section ****************************/

				// TODO : add sliding bar to animate Thrust.
				// TODO : use kalman filter for that phase. to make it smooth
				if (system_state == StereoRobotApp::TRACKING)
				{
					double alpha =0;
					double thrust_per=0;
					if ( (effective_depth_measurement > minDepth_toGoTo) && 
						 (effective_depth_measurement < maxDepth_toGoTo) ) // 5 as minimum depth to go to
					{
						alpha = tracked_target.target_estimated_dx * myStereoCams.camFOVpix ;
						//double thrust_per = (avg_depth_of_ROI-Dmin)/(Dmax-Dmin);
						thrust_per = 100.0*(effective_depth_measurement-11)/(190-11);
						hardwareController.Forward(thrust_per,  alpha , 0.9);
					}
					else 
					{
						hardwareController.Stop();
					}

					////////////// added graphics section /////////// 
					if (op_flags.draw_middle_x)
					{			
						Point2f targetCenter ;
						// TODO: test source of track errors, from BackgroundSubs, or Tracker
						///	add_Cross_to_Image(tracker.TrkErrX  ,  left_im_color.size().height/2  , 
						targetCenter	=	tracker.MassCenter;
						myGUI.add_Cross_to_Image(targetCenter.x  ,  targetCenter.y  , 
							true, system_state , left_im_color); //	changes input image. but it is last in the flow cycle.
					}

				}
				else
				{
					hardwareController.Stop();
				}


				/************************************************/

				myGUI.show_disparity_map(sum_of_N_disparities, effective_depth_measurement);

			}											

			// move to broadcast from outside. // separate the graphic layer to only when track is on.
			//									otherwise just draw raw images.

			myGUI.show_raw_captures(left_im_color, right_im_color, myStereoCams.GetFrameCycleCounter(), system_state);

			////////////* end of graphics section *///////////////
		}

		/////////////////////////////////////////////////////////////////////
		///////////////////////user response handling////////////////////////
		/////////////////////////////////////////////////////////////////////
		
		if (wait_or_handle_user_input())
			break;
	}

}

bool StereoRobotApp::wait_or_handle_user_input()
{
	bool return_value = false;

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
	
	//double thrust_percent, 
	double angle;
	switch (c) {
	case StereoRobotApp::UNIX_KEY_UP: 
		angle = 0;			
		hardwareController.Forward(userFwdThrust_percent, angle, 0);
		if (userFwdThrust_percent <= 90)
			userFwdThrust_percent += 10 ;
		break;
	case StereoRobotApp::UNIX_KEY_DOWN:
		hardwareController.Stop();
		userFwdThrust_percent	=	initialUserFwdThrust_percent;
		break; 
	case StereoRobotApp::UNIX_KEY_LEFT: 
		angle = -45 /57.3;		 //-45deg
		hardwareController.Forward(userFwdThrust_percent, angle, 0.9);	// 0.9 is turnRateRatio
		break;
	case StereoRobotApp::UNIX_KEY_RIGHT: 
		angle = 45 /57.3;		
		hardwareController.Forward(userFwdThrust_percent, angle, 0.9);
		break;
	}

	if (c==27)
		return_value = true;

	return return_value;
}