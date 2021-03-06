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

	working_FRAME_WIDTH		=	320 ;	working_FRAME_HIGHT		=	240 ;	working_FRAMES_FPS		=	15;
	///working_FRAME_WIDTH		=	160 ;	working_FRAME_HIGHT		=	120 ;	working_FRAMES_FPS		=	15;
	///working_FRAME_WIDTH		=	160 ;	working_FRAME_HIGHT		=	120 ;	working_FRAMES_FPS		=	30;
	///working_FRAME_WIDTH		=	 80 ;	working_FRAME_HIGHT		=	 60 ;	working_FRAMES_FPS		=	30;

	frame_boundary_W_init	=	0;
	frame_boundary			=	0;

	minDepth_toGoTo			=	25;	//[cm]
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

	userFwdThrust_percent	=	initialUserFwdThrust_percent;
	MovementNoTrackCycles	=	0;
	MovementNoTrackCycles_Allowed = 15;	//15 for 320x240, constraign that up to 15 cycle from movement to lock on target
										// otherwise - probobly no valid target
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
	int64 tMain = getTickCount(); 

	while (1)		
	{

		tMain = getTickCount();

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
			if ( (system_state == INITIALIZING) || (system_state == STANDBY) 
				|| (system_state == FOUND_SOME_MOVEMENT) || (system_state == TARGET_IS_LOST) )
			{
				localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
																								 //
				if (localBackSubs.BgSubt_Status == BackSubs::BG_STANDING_BY)
					system_state = StereoRobotApp::STANDBY; 
				else 
					if (localBackSubs.BgSubt_Status == BackSubs::BG_FOUND_MOVEMENT)
					{
						system_state	= StereoRobotApp::FOUND_SOME_MOVEMENT;
						featTrackMask_fromBgSubt	= localBackSubs.get_foreground_mat(); 
					}					
			}
			else
			{
				localBackSubs.show_forgnd_and_bgnd_init(working_FRAMES_FPS, false);
				featTrackMask_fromBgSubt	= localBackSubs.get_foreground_mat();	// return empty Mat for this phase
				myGUI.close_BgSubt_win();  // TODO: just add red frame, and when operative set to green
			}
			
			if ( (system_state == FOUND_SOME_MOVEMENT) || (system_state == TRACKING) )
			{
				/************************disp************************/
				localDisp.calc_disperity(1, left_im_gray, right_im_gray, featTrackMask_fromBgSubt, &tracked_target,
											&current_disparity , &last_min_depth_of_ROI ); // 3-also filters

				if (!myGUI.bSHOW_as_demo_movie_flow)
					myGUI.show_disparity_map(current_disparity, last_min_depth_of_ROI, 3); //newly added

				localDisp.calc_disperity(2, left_im_gray, right_im_gray, featTrackMask_fromBgSubt, &tracked_target,
											&current_disparity , &last_min_depth_of_ROI ); // als
				
				if (!myGUI.bSHOW_as_demo_movie_flow)
					myGUI.show_disparity_map(current_disparity, last_min_depth_of_ROI, 2); //newly added

				matQueue.populateNextElementInArray(current_disparity); 
				matQueue.getSumElement(&sum_of_N_disparities);

				doubleQueue.populateNextElementInArray(last_min_depth_of_ROI);
				doubleQueue.getAvgElement(&avg_depth_of_ROI);

				featTrackMask_fromDisperity	=	sum_of_N_disparities.clone();		// sum of last 3 frames
				myGUI.show_disparity_map(featTrackMask_fromDisperity, avg_depth_of_ROI, 1); //newly added
					 	//TODO: drive should be around D closer. and around ROI.  in tracker - dont drag FP that are not in D ROI ( or vrery far..)
				threshold (featTrackMask_fromDisperity , featTrackMask_fromDisperity ,	1 ,	255,THRESH_BINARY);	// take all that is not zero 
				effective_depth_measurement		=	avg_depth_of_ROI;		// rounded to [cm]

			//	tracked_target.target_object_prop.relevant_disparity =  take current or average??
				tracked_target.target_object_prop.target_estimated_distance = effective_depth_measurement; // taking the average

				/********************feat.tracker****************************/
				  
				featTrackMask	= Mat::zeros(left_im_gray.size(), left_im_gray.type() ) ; 
				if (system_state == StereoRobotApp::FOUND_SOME_MOVEMENT)
				{
					//featTrackMask_fromBgSubt.copyTo (featTrackMask, featTrackMask_fromDisperity ) ;	// output is featTrackMask , by mask1 && mask2
					featTrackMask = featTrackMask_fromDisperity.clone(); // combine with BgSubt already in disp
					MovementNoTrackCycles++;
				}
				else
					featTrackMask = featTrackMask_fromDisperity.clone(); // TODO:TODO: need to combine with previous FP ROI

				myGUI.printFPinputMask(featTrackMask);
				//	mainImSeg(left_im_color);	// need to use the markers with depth ROI anti-mask as sure background.
				// or use the inside use of distanceTransform to better to find the bigger object.

				trackerNotOff = false;
				if ( tracker.Tracker_State != Tracker::TRACKER_OFF)
					trackerNotOff = true;

				tracked_target.set_target_mask_properties(featTrackMask);
				tracker.processImage(left_im_gray, &tracked_target) ;	//tracked_target is IN/OUT object

#ifdef COMPILING_ON_ROBOT
			///	vector<Point2f> stam;
			///	tracker.display_allFPoints(false, stam);	// false for debug prints
#endif

				if ( (trackerNotOff) && (tracker.Tracker_State == Tracker::TRACKER_OFF) )	//back from advanced mode to OFF
				{
					system_state	=	 StereoRobotApp::TARGET_IS_LOST ;
					localBackSubs.show_forgnd_and_bgnd_init(0,true);
					myGUI.close_Tracking_win();
				}
				if ( tracker.Tracker_State == Tracker::TRACKER_TRACKING )	//back from advanced mode to OFF
				{
					system_state	=	 StereoRobotApp::TRACKING ; 
					//continue;	
				}

				if (MovementNoTrackCycles > MovementNoTrackCycles_Allowed)
				{
					MovementNoTrackCycles = 0;
					system_state = StereoRobotApp::STANDBY; 
					localBackSubs.show_forgnd_and_bgnd_init(0,false);
					myGUI.close_Tracking_win();
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
						///alpha = tracked_target.target_estimated_dx * myStereoCams.camFOVpix ;
						alpha = (double) tracked_target.target_object_prop.target_estimated_dx / w ;
						//double thrust_per = (avg_depth_of_ROI-Dmin)/(Dmax-Dmin);
						thrust_per = 100.0*(effective_depth_measurement-minDepth_toGoTo)/(maxDepth_toGoTo-minDepth_toGoTo);
						hardwareController.Forward(thrust_per,  alpha);
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
							true, system_state , left_im_color,
							tracker.trackedTarget.target_mask_prop.boundRect); //	changes input image. but it is last in the flow cycle.
					}

				}
				else
				{
					hardwareController.Stop();
				}


				/************************************************/
			}											

			// move to broadcast from outside. // separate the graphic layer to only when track is on.
			//									otherwise just draw raw images.

			myGUI.show_raw_captures(left_im_color, right_im_color, myStereoCams.GetFrameCycleCounter(), system_state);

			if (myGUI.bUseExternalRecordedFile)
			{
				myGUI.showExternalVideoFrame(myStereoCams.GetRes().width, myStereoCams.GetRes().height);

				if (myGUI.bSHOW_as_demo_movie_flow)
				{ 
					Mat tmp =Mat();

					IplImage mat_arr[myGUI_handler::NUM_OF_GUI_WINDOWS];
					IplImage *mat_arrP[myGUI_handler::NUM_OF_GUI_WINDOWS];

					for (int i=0; i< myGUI_handler::NUM_OF_GUI_WINDOWS; i++)
					{  
						if ((i==2) || (i==3) || (i==5) )
						{ 
						vector<Mat> tmp_channels;
							tmp_channels.push_back(myGUI.plotImages[i]);
							tmp_channels.push_back(myGUI.plotImages[i]);
							tmp_channels.push_back(myGUI.plotImages[i]);
							merge(tmp_channels, tmp);
							mat_arr[i] = (IplImage)tmp;

						}
						else
							mat_arr[i] = (IplImage)myGUI.plotImages[i];
						mat_arrP[i] = cvCloneImage(&mat_arr[i]); 
					}

					cvShowManyImages(
						"all images" ,	myGUI.NUM_OF_GUI_WINDOWS , 						
											mat_arrP[myGUI_handler::WIN2_NDX_LeftRawIm],					
											mat_arrP[myGUI_handler::WIN1_NDX_RightRawIm],
											mat_arrP[myGUI_handler::WIN7_NDX_ExternalRecord],
											mat_arrP[myGUI_handler::WIN3_NDX_BgSubtMask],
											mat_arrP[myGUI_handler::WIN4_NDX_DisparityMask],
											mat_arrP[myGUI_handler::WIN6_NDX_FPinptMask],
											mat_arrP[myGUI_handler::WIN5_NDX_FeaturePoints]
					);
				}
			}

			////////////* end of graphics section *///////////////
		}

		/////////////////////////////////////////////////////////////////////
		///////////////////////user response handling////////////////////////
		/////////////////////////////////////////////////////////////////////
		
		tMain = getTickCount() - tMain;
		printf("main cycle Time elapsed: %fms\n\n", tMain*1000/getTickFrequency());

		if (wait_or_handle_user_input())
			break;
	}

	hardwareController.Stop();
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
		if (!myGUI.bSHOW_as_demo_movie_flow)
			c = waitKey(0*loop_delay);
		else	
			waitKey( 100 );//extra waiting time for the user to check all display windows

		myStereoCams.ToggleDisableFramesCapture();
		waitKey( loop_delay );
	}
	
	//double thrust_percent, 
	double angle;
	switch (c) {
	case StereoRobotApp::UNIX_KEY_UP: 
		angle = 0;			
		hardwareController.Forward(userFwdThrust_percent, angle);
		if (userFwdThrust_percent <= 90)
			userFwdThrust_percent += 10 ;
		break;
	case StereoRobotApp::UNIX_KEY_DOWN:
		hardwareController.Stop();
		userFwdThrust_percent	=	initialUserFwdThrust_percent;
		break; 
	case StereoRobotApp::UNIX_KEY_LEFT: 
		angle = -45 /57.3;		 //-45deg
		hardwareController.Forward(userFwdThrust_percent, angle);	// 0.9 is turnRateRatio
		break;
	case StereoRobotApp::UNIX_KEY_RIGHT: 
		angle = 45 /57.3;		
		hardwareController.Forward(userFwdThrust_percent, angle);
		break;
	}

	if (c==27)
		return_value = true;

	return return_value;
}
