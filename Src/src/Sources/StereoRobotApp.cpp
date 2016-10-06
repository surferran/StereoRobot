/*
 *  StereoRobotApp.cpp  
 */

#include "..\Headers\StereoRobotApp.hpp"

 /**************************************/
#include "myGUI_handler.h"
myGUI_handler		myGUI;

ImagesSourceHandler myStereoCams; // thread for images capturing

Target				first_target ;

Tracker				tracker;
Target::TargetState tmpTargStat;

Depth_and_Disparity							localDisp;

Depth_and_Disparity::rectification_outputs	disperity_struct;

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

};


StereoRobotApp::~StereoRobotApp(){};

/************************************************************/

void StereoRobotApp::appInitializations()
{
	op_flags.show_stereo=true;	// initialize and conduct stereo algo imidiatly when running.

	user_pressing	= 0;	// just optional.
	got_1st_stable_bkgnd = false;

	w		= myStereoCams.GetRes().width;
	h		= myStereoCams.GetRes().height;

	localBackSubs.show_forgnd_and_bgnd_init(working_FRAMES_FPS); //vidL//with Left cam  

	/* clear points that are out of my desired ROI (center of image) */
	//TODO:make 20 h , 30 w /// sizes are for after resize
	Point	TopLeft(frame_boundary, frame_boundary); 
	Point	LowRight(w - frame_boundary , h - frame_boundary);
	TrackingROI = Rect(TopLeft, LowRight ); 

	//	ROI for background substraction is narrower then the one for the tracker
	TopLeft		 = Point(frame_boundary_W_init		, frame_boundary	); 
	LowRight	 = Point(w - frame_boundary_W_init	, h - frame_boundary);
	BckgndSubROI = Rect(TopLeft, LowRight ); 
}

void StereoRobotApp::appMainLoop()
{

	while (1)		
	{
/*
#ifndef COMPILING_ON_ROBOT
		if (op_flags.make_stereo_calibration)
		{		
			argc = 6;
			argv[1] = "-w";  argv[2] = "8";
			argv[3] = "-h";  argv[4] = "6";
			argv[5] = "../run_inputs/stereo_calibration_images/stereo_calib.xml";
			do_stereo_calib(argc, argv);
			op_flags.make_stereo_calibration	=	false;
		}
#endif*/

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

			if (myStereoCams.GetUserRepeatFlag())
			{
				// if testing recorded files - don't continue to read image frames in background. 
				// until next deterministic loop request. (specially if stopping for debugging).
				myStereoCams.ToggleDisableFramesCapture();
			}

			////////////// end of capture images ///////////

			myGUI.show_raw_captures(left_im_color, right_im_color, myStereoCams.GetFrameCycleCounter());

			/************************bgSubt************************/
			if ( (system_state == INITIALIZING) || (system_state == STANDBY) || (system_state == FOUND_SOME_MOVEMENT) )
			{
				localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
																								 //
				if (localBackSubs.BgSubt_Status == BackSubs::STANDING_BY)
					system_state = StereoRobotApp::STANDBY; 
				else if (localBackSubs.BgSubt_Status == BackSubs::FOUND_MOVEMENT)
						system_state = StereoRobotApp::FOUND_SOME_MOVEMENT;
			}
			
			/************************disp************************/
			if ( (system_state == FOUND_SOME_MOVEMENT) || (system_state == FOUND_GOOD_TARGET) )
			{
				localDisp.calc_disperity(3, left_im_gray, right_im_gray, &current_disparity , &last_min_depth_of_ROI );  //2 , when 1 above

				matQueue.populateNextElementInArray(current_disparity); 
				matQueue.getSumElement(&sum_of_N_disparities);

				doubleQueue.populateNextElementInArray(last_min_depth_of_ROI);
				doubleQueue.getAvgElement(&avg_depth_of_ROI);
			}

			switch (system_state) 
			{
			case StereoRobotApp::FOUND_SOME_MOVEMENT:
			case StereoRobotApp::FOUND_GOOD_TARGET:		///
														
														
			
				/********************feat.tracker****************************/

				int depth;		// rounded to [cm]
								//
				featTrackMask	= Mat::zeros(left_im_gray.size(), left_im_gray.type() ) ;
				if (system_state == StereoRobotApp::FOUND_SOME_MOVEMENT)
					featTrackMask1	= localBackSubs.get_foreground_mat(); 
				featTrackMask2	=	sum_of_N_disparities.clone();		// sum of last 3 frames
																//TODO: drive should be around D closer. and around ROI.  in tracker - dont drag FP that are not in D ROI ( or vrery far..)
				threshold (featTrackMask2 , featTrackMask2 ,	1 ,	255,THRESH_BINARY);		// take all that is not zero THRESH_TOZERO
				depth			=	avg_depth_of_ROI;
				//

				if (system_state == StereoRobotApp::FOUND_SOME_MOVEMENT)
					featTrackMask1.copyTo (featTrackMask, featTrackMask2 ) ;	// output is featTrackMask , by mask1 && mask2
				else
					featTrackMask = featTrackMask2.clone(); // TODO:TODO: need to combine with previous FP ROI

															///featTrackMask = featTrackMask2.clone();
															///threshold (featTrackMask , featTrackMask ,	1 ,	255,THRESH_BINARY);		// take all that is not zero THRESH_TOZERO

															////////////////////////

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
/*
				cv::cvtColor(left_im_color , left_im_gray  , CV_BGR2GRAY);
				cv::cvtColor(right_im_color, right_im_gray , CV_BGR2GRAY);*/

				///	mainImSeg(left_im_color);	// need to use the markers with depth ROI anti-mask as sure background.
				// or use the inside use of distanceTransform to better to find the bigger object.

				//// tmpROI is MASK in potential area of new target. 
				tmpROI							=  Mat::zeros( left_im_gray.size() , left_im_gray.type() );
				first_target.potential_target	= tmpROI.clone();

				tmpROI( first_target.target_mask_prop.boundRect ) = 255;
				// potential target is Trimmed area acording to ROI (from Depth or BgSubt)					
				left_im_gray.copyTo(first_target.potential_target , tmpROI); 

				trackerNotOff = false;
				if ( tracker.Tracker_State != Tracker::TRACKER_OFF)
					trackerNotOff = true;

				/* *********** */	tracker.processImage(left_im_gray, first_target.potential_target, first_target.target_mask_prop.boundRect );// localBackSubs.get_foreground_boundRect() );

				if ( (trackerNotOff) && (tracker.Tracker_State == Tracker::TRACKER_OFF) )	//back from advanced mode to OFF
				{
					system_state	=	 StereoRobotApp::INITIALIZING ;
					//TODO: init BackSubs.BgSubt_Status to init 
					//TODO: close potential_target window
					break ;
				}
				if ( tracker.Tracker_State == Tracker::TRACKER_TRACKING )	//back from advanced mode to OFF
				{
					system_state	=	 StereoRobotApp::FOUND_GOOD_TARGET ; 
					break ;
				}

				// center is by MaskROI. not related to feature points.. TODO: fix that.
				// more: empty mask - not affecting. need to cut feature points in trcker by mask. by phase of D only or using BGsubt.
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

				// track forward

				// TODO : add sliding bar to animate Thrust.
				// TODO : use kalman filter for that phase. to make it smooth
				if (system_state == StereoRobotApp::FOUND_GOOD_TARGET)
				{
					if ( (avg_depth_of_ROI > minDepth_toGoTo) && 
						(avg_depth_of_ROI < maxDepth_toGoTo) ) // 5 as minimum depth to go to
					{
						//thrust_percent = 50; //make static somwhere
						//angle = -45 /57.3;		 //-45deg
						//double thrust_per = (avg_depth_of_ROI-Dmin)/(Dmax-Dmin);
						double thrust_per = 100.0*(avg_depth_of_ROI-11)/(190-11)*5.0;
						hardwareController.Forward(thrust_per,  0, 0);
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
							false, system_state , left_im_color); //	changes input image. but it is last in the flow cycle.
					}

				}
				else
				{
					hardwareController.Stop();
				}


				/************************************************/

				myGUI.show_disparity_map(sum_of_N_disparities, avg_depth_of_ROI);
				
				break;

			}											

			// move to broadcast from outside. // separate the graphic layer to only when track is on.
			//									otherwise just draw raw images.

			myGUI.show_raw_captures(left_im_color, right_im_color, myStereoCams.GetFrameCycleCounter());

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
		return_value = true;

	return return_value;
}