//// 3D stero robot (RoboDog)
/*
	history can be vied in the GitHub site repository.
	written app : by Ran , year 2016
*/


#define MANUAL_TESTINGS		false // true - to allow manual overides to steering (by arrow-keys), and selecting target (by mouse)

/*******************************************************************************************/
/*******************************************************************************************/
 /* my  constants and parameters */

// bounds in percent from image size
#define MIN_MOVED_AREA_in_image 33//33//.0//23
#define MAX_MOVED_AREA_in_image 90.0  //95
 //#define NUM_OF_PIXELS_IN_FRAME	(640.0*480.0)
#define NUM_OF_PIXELS_IN_FRAME	(working_FRAME_WIDTH * working_FRAME_HIGHT)
#define MIN_CURVE_AREA			MIN_MOVED_AREA_in_image/100.0*NUM_OF_PIXELS_IN_FRAME
#define MAX_CURVE_AREA			MAX_MOVED_AREA_in_image/100.0*NUM_OF_PIXELS_IN_FRAME
#define SHOW_MOVING_CONTOURS		true//true
#define SHOW_MOVING_RECTANGLES		false//true
#define SHOW_MOVING_BIG_CONTOURS	true
#define SHOW_MOVING_BIG_RECTANGLES	false//true

// below or above those limits it will be treated as noise. (or too close movement to camera)
/*******************************************************************************************/
/*******************************************************************************************/

/*********  thread object  ********/
#include "stereo_functions.hpp"		//set the disparity object (variables and functions)
myLocalDisparity localDisp;
/**********************************/

#include "frameFunctions.h"		// general definitions and functions. that's why it is first to include.
#include "working_consts.h"		// my added definitions, constants

///StereoCams		thisStereo;				// global 
SYSTEM_STATUS	system_state = INITIALIZING ;
Operation_flags	op_flags; //global

#include "BackgroundSub.hpp"
#include "OdroidC1_handlers\RobotController.h"

#include "stereo_calib.h" 
#include "FeatureTracker.hpp"
#include "ImagesSourceHandler.h"
/*********  GUI object  ********/
#include "myGUI_handler.h"
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

#include "imporeted_raw_code_examples/watershed_from_OpencvSamples.cpp"
#include "imporeted_raw_code_examples/backSubsExample.cpp"
#include "imporeted_raw_code_examples/grabcut_from_OpencvSamples.cpp"//


////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////					  main	   			  //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////// 

int main(int argc, char** argv) 
{
 	/* variables */
	Mat left_im_color ,
		right_im_color;
	Mat left_im_gray ,
		right_im_gray;

	///thisStereo.input_source = LIVE_CAM;

#ifdef COMPILING_ON_ROBOT
	RobotController hardwareController ;
#endif

	op_flags.show_stereo=true;	// initialize and conduct stereo algo imidiatly when running.

///	string	base_out_file_path	= "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/out";
	//string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".*";	
	//int		stream_frame_index	= 0;
	/*char	rec_file_name[150]  = "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/in/VID_3D_scenario/output_1.avi";
	VideoCapture vid			= VideoCapture(rec_file_name);	*/
	Mat		target_candidate_features;
	///Mat		tracked_target_image;

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
	const int target_lost_timeout_counter  = 2* loop_delay ; // [~sec]//counter to simulate delay of about 2 sec. (depend on loop inner delay)
	int		  target_lost_time_counter = 0 ;   // stopper to timeout

	Tracker		tracker;
	Rect		BckgndSubROI;
	Rect		TrackingROI;

	Mat			lastDepthImg,
				depthAvg;
	int			depthAvgNdx = 0;

	bool got_1st_stable_bkgnd = false;

	myLocalDisparity::rectification_outputs disperity_struct;
	Mat		disp_temporary;
	Mat		modified_disperity_mat;
	Scalar	avg_disperity_S;
	double	avg_disperity;
	double	avg_depth_of_ROI ;

	Point	movementMassCenter, corected_MassCenter;
	Mat		bgnd;

	bool	gotNewDispImageToWorkWith = false;

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
	Point	TopLeft(frame_boundary, frame_boundary); 
	Point	LowRight(w - frame_boundary , h - frame_boundary);
	TrackingROI = Rect(TopLeft, LowRight ); 

	//	ROI for background substraction is narrower then the one for the tracker
	TopLeft		 = Point(frame_boundary_W_init		, frame_boundary	); 
	LowRight	 = Point(w - frame_boundary_W_init	, h - frame_boundary);
	BckgndSubROI = Rect(TopLeft, LowRight ); 

	////////////// end of initializations ///////////
	
	while (1)		
	{

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
#endif

#ifndef COMPILING_ON_ROBOT
		if(op_flags.show_stereo)
#endif
		{
			////////////// capture images ///////////
			myStereoCams.GetFrames(myGUI.plotImages[0],myGUI.plotImages[1]);
			if ((myGUI.plotImages[0].empty()) )
				continue;
			if ((myGUI.plotImages[1].empty()) )
				continue;
			
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
			
			//// example code
			//mainGC(left_im_color);
			////mainWSH(left_im_color);	//NICE one for fast segmenting
			/*
			myWaterShed WSH;
			WSH.init_mask_by_input (left_im_color);
			*/
			//return 0;

			////////////*////////////*////////////*////////////*////////////
			////////////* get Disperity & DEPTH by Stereo */////////////// 
			if (!RUN_ON_LAPTOP__MONO)
			{
				// calc disparity every 1, 2 frame
				if (
					(relative_counter > (localDisp.calcDispEveryNcycles - 1)) &&    //10  
					(system_state > STANDBY) 
					) 
				{ 
					/* sends gray images */
					cv::cvtColor(left_im_color , left_im_gray  , CV_BGR2GRAY);
					cv::cvtColor(right_im_color, right_im_gray , CV_BGR2GRAY);

					// delivers new input , when the process is waiting (not in calculation process)
					localDisp.set_disparity_input(right_im_gray,left_im_gray, myStereoCams.GetFrameCycleCounter() );  
					
					/* if output is ready from disparity calculation , it returns true */
					if ( localDisp.get_rectified_and_disparity(disp_temporary, disperity_struct) )  
					{
						myGUI.plotImages[2]    = disp_temporary;	// keep for display

						/////////////
						/* calculate average depth for the ROI of the target */
						threshold (disp_temporary , modified_disperity_mat ,	50 ,	255,THRESH_TOZERO);/// THRESH_BINARY

						avg_disperity_S = mean( modified_disperity_mat((tracker.current_trackingROI)) ) ;
						avg_disperity	= avg_disperity_S[0];

						localDisp.convert_disperity_value_to_depth(avg_disperity , avg_depth_of_ROI);
						///			cout   << " avg_disperity " << avg_disperity  << " avg_depth_of_ROI " << avg_depth_of_ROI << endl;
						////////////

						/* displays */
					/*	myGUI.display_rectified_pair( disperity_struct.imageSize , disperity_struct.rectR , disperity_struct.rectL, 
														disperity_struct.validROI1 , disperity_struct.validROI2 , 
															disperity_struct.originalyGivenframeCycle );
*/
						imshow(myGUI.plotWindowsNames[2],  myGUI.plotImages[2]);

						lastDepthImg		= modified_disperity_mat.clone();

						int an=3;	//an=1->kernel of 3
						Mat element = getStructuringElement(MORPH_RECT, Size(an*2+1, an*2+1), Point(an, an) );
						medianBlur	(lastDepthImg,	lastDepthImg,	9);//9//3
						erode		(lastDepthImg , lastDepthImg, element);									
						dilate		(lastDepthImg,	lastDepthImg, element);
						Mat depthMask = lastDepthImg.clone();

						imshow ( myGUI.plotWindowsNames[7], lastDepthImg);
						///cvtColor(depthMask	, depthMask , COLOR_BGR2GRAY);

						gotNewDispImageToWorkWith = true;

						request_water_shed	= true;			//segmentation
						 
					}
					relative_counter	=	0;
				}
			}
			////////////* end of -get Disperity & DEPTH by stereo *///////////////
			////////////*////////////*////////////*////////////*////////////

			//enum SYSTEM_STATUS{
			//	INITIALIZING	=	0 ,		// Should show GRAY cross	 
			//	STANDBY			=	1 ,		// Should show ORANGE cross
			//	FOUND_SOME_MOVEMENT	,
			//	FOUND_GOOD_TARGET	,
			//	TRACKING_GOOD_QUALITY_TARGET,		// Should show GREEN cross
			//	TRACKING_LOW_QUALITY_TARGET,
			//	TARGET_IS_LOST			// Should show RED cross	,	after 3 sec will turn to ORANGE (while stopping the robot)
			//};
			Mat potential_target;

			switch (system_state) 
			{
				case INITIALIZING:
				case STANDBY:
				case FOUND_SOME_MOVEMENT: 

					/* run BackGroundSubs as long as searching for GoodTarget (system_state is FOUND_GOOD_TARGET) */
					
					localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
					if (system_state <= STANDBY) /// INITIALIZING)
					{
						if (myStereoCams.GetUserRepeatFlag())
						{
							myStereoCams.ToggleDisableFramesCapture();
						}
						int c2 = waitKey(loop_delay);
						if (c2==27)
							break;
						continue;
					}

					//else	// system_state == FOUND_SOME_MOVEMENT
					
					corected_MassCenter = Point(movementMassCenter.x + BckgndSubROI.x,  movementMassCenter.y + BckgndSubROI.y);
					////actually not needed .. makeContours(localBackSubs.get_foreground_mat()); 
											
					left_im_color.copyTo		//left_im_color.(localBackSubs.get_foreground_boundRect())
										( potential_target, localBackSubs.get_foreground_mat() ) ;
					tracker.processImage(left_im_gray, potential_target, system_state , localBackSubs.get_foreground_boundRect() );

					circle(potential_target, corected_MassCenter, 4, Scalar(0, 255, 255), -1, 8, 0);
					imshow(myGUI.plotWindowsNames[8], potential_target);
					 
					break;

				case FOUND_GOOD_TARGET:
					break;
			}				

			////////////////////////////////////////////////
			//			tracking part (by 'goodFeatures')
			if ( system_state == FOUND_GOOD_TARGET )
			{
				Mat mat1 = localBackSubs.get_the_background_average() ;
				Mat mat2 = localBackSubs.get_foreground_mat() ;
				Mat bgndDiff ;
				//bgndDiff = mat1  -  mat2 ;
				//imshow("BackSubs bg-fg diff",bgndDiff); //debugging

				bgndDiff = mat1  -  left_im_color ;
				///imshow("BackSubs bg-current diff",bgndDiff); //debugging

				//if ( request_water_shed )
				//{
				//	WSH.calculate_the_watershed(lastDepthImg);//    localBackSubs.get_foreground_mat());
				//	request_water_shed = false;
				//}
				////consider GrabCut because input is spreaded points , and not curves

				// want to init and lock the tracker
				// get the feature points of the target from the BackgroundSubs ROI
				Mat tracked_target_image;
				Rect corrected_ROI = Rect(	BckgndSubROI.x  + localBackSubs.get_foreground_boundRect().x ,
											BckgndSubROI.y  + localBackSubs.get_foreground_boundRect().y ,
											localBackSubs.get_foreground_boundRect().width ,
											localBackSubs.get_foreground_boundRect().height ) ;

				/* keep also original target depth image */
				if (!RUN_ON_LAPTOP__MONO)
				{
					myGUI.plotImages[5] = myGUI.plotImages[2]  ;	//last depth

					myGUI.plotImages[5](BckgndSubROI).copyTo(myGUI.plotImages[6], localBackSubs.get_foreground_mat() );
					imshow(myGUI.plotWindowsNames[5],  myGUI.plotImages[5]);
					imshow(myGUI.plotWindowsNames[6],  myGUI.plotImages[6]);
				}
				left_im_color(corrected_ROI).copyTo(tracked_target_image);
				imshow("tracked Target start", tracked_target_image) ; // show 4 debug  only

				tracker.setNewTarget(corrected_ROI, tracked_target_image, TrackingROI);

				// get smooth disparity and 
				//TODO: clean the far points - disperity + watershed

				if ( request_water_shed )
				{
					//WSH.calculate_the_watershed(lastDepthImg);//    localBackSubs.get_foreground_mat());
					//request_water_shed = false;
				}
			}
			Point2f targetCenter ;

			////////////// added graphics section ///////////
			left_im_color	= myGUI.plotImages[1].clone();   // for some additional display layer

			if (op_flags.draw_middle_x)
			{
				//on the right image
				//add_Cross_to_Image(left_im_color.size[1]/2, left_im_color.size[0]/2, false, system_state , left_im_color); // 120h,160w , with no coor. label

				// TODO: test source of track errors, from BackgroundSubs, or Tracker
			///	add_Cross_to_Image(tracker.TrkErrX  ,  left_im_color.size().height/2  , 
				myGUI.add_Cross_to_Image(targetCenter.x  ,  targetCenter.y  , 
										false, system_state , left_im_color); // 120h,160w , with no coor. label
					
			}
				
			////////////* end of graphics section *///////////////
			
			// move to broadcast from outside. // separate the graphic layer to only when track is on.
			//									otherwise just draw raw images.

			long tmpLong = myStereoCams.GetFrameCycleCounter();
			myGUI.add_counterFrame(left_im_color , &tmpLong ) ;
			imshow(myGUI.plotWindowsNames[0],	myGUI.plotImages[0] );
			imshow(myGUI.plotWindowsNames[1],	left_im_color);
			 
		}

		/////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////
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
			waitKey(loop_delay);
		}

		if (c==(int)'w')
			request_water_shed = true;

#ifdef COMPILING_ON_ROBOT
		switch (c) {
		case KEY_UP:
			double thrust_percent = 50; //make static somwhere
			double angle = 0;			
			hardwareController.Forward(thrust_percent, angle, 0);
			break;
		case KEY_DOWN:
			hardwareController.Stop();
			break; 
		case KEY_LEFT:
			double thrust_percent = 50; //make static somwhere
			double angle = -45 /57.3;		 //-45deg
			hardwareController.Forward(thrust_percent, angle, 0.9);
			break;
		case KEY_RIGHT:
			double thrust_percent = 50; //make static somwhere
			double angle = 45 /57.3;		
			hardwareController.Forward(thrust_percent, angle, 0.9);
			break;
		}
#endif
		if (c==27)
			break;
		/////cout << " c " << (int)c <<endl;
		//if (!check_user_input(&loop_delay, &user_pressing))
			//break;
	}
	
	return 0;

}