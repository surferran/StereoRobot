//// 3D stero robot (RoboDog)
/*
	history can be vied in the GitHub site repository.
	written app : by Ran , year 2016
*/


#define MANUAL_TESTINGS		false // true - to allow manual overides to steering (by arrow-keys), and selecting target (by mouse)

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

/*********  thread object  ********/
#include "stereo_functions.hpp"		//set the disparity object (variables and functions)
myLocalDisparity localDisp;
/**********************************/

#include "frameFunctions.h"		// general definitions and functions. that's why it is first to include.
#include "working_consts.h"		// my added definitions, constants

StereoCams		thisStereo;				// global 
SYSTEM_STATUS	system_state = INITIALIZING ;
Operation_flags	op_flags; //global

#include "BackgroundSub.hpp"
#include "OdroidC1_handlers\RobotController.h"

#include "stereo_calib.h" 
#include "FeatureTracker.hpp"

#ifdef COMPILING_ON_ROBOT
////#include "OdroidC1_handlers/RobotController.h"
#include <unistd.h>

#include "pwm.h"
#endif

#include <mutex>
#include <atomic> 
#include <memory>
#include <thread>

#include "ImagesSourceHandler.h"
#include "myGUI_handler.h"

/*  initiating GUI setup*/
myGUI_handler myGUI; // thread for images displaying
/*  ***************************  */

//#include "showManyImages.cpp"   ///void cvShowManyImages(char* title, int nArgs, ...) ;

////when need to eliminate the consule that is opened in parallel 
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

//#include "grabcut_from_OpencvSamples.cpp"

#include "imporeted_raw_code_examples/watershed_from_OpencvSamples.cpp"
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

	thisStereo.input_source = LIVE_CAM;

#ifdef COMPILING_ON_ROBOT
	RobotController hardwareController ;

#endif

	op_flags.show_stereo=true;	// initialize and conduct stereo algo imidiatly when running.

///	string	base_out_file_path	= "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/out";
	string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".*";	
	int		stream_frame_index	= 0;
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

	const int target_lost_timeout_counter  = 2* loop_delay ; // [~sec]//counter to simulate delay of about 2 sec. (depend on loop inner delay)
	int		  target_lost_time_counter = 0 ;   // stopper to timeout


	/*  initiating createBackgroundSubtractorMOG2   */
	BackSubs	localBackSubs ;
	/*  *****************************************   */

	Tracker		tracker;
	Rect		BckgndSubROI;
	Rect		TrackingROI;

	Mat			lastDepthImg,
				depthAvg;
	int			depthAvgNdx = 0;

	bool got_1st_stable_bkgnd = false;

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
	
	//TODO:  (default image target shoult be zeroes(res.x, res.y) )
	//		allow 1 or 2 frames to be with no featres. set a GapCounter.

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
			////////////// end of capture images ///////////
			
			//// example code
			//mainGC(left_im_color);
			////mainWSH(left_im_color);	//NICE one for fast segmenting
			myWaterShed WSH;
			WSH.init_mask_by_input (left_im_color);
			
			//return 0;
			/////

			myLocalDisparity::rectification_outputs disperity_struct;
			Mat		disp_temporary;
			Mat		modified_disperity_mat;
			Scalar	avg_disperity_S;
			double	avg_disperity;
			double	avg_depth_of_ROI ;

			////////////* get Disperity & DEPTH by Stereo */////////////// 
			if (!RUN_ON_LAPTOP__MONO)
			{
				// calc disparity every 1, 2 frame
				if (relative_counter>0) //10  
				{ 
					/*myLocalDisparity::rectification_outputs disperity_struct;
					Mat		disp_temporary;
					Mat		modified_disperity_mat;
					Scalar	avg_disperity_S;
					double	avg_disperity;
					double	avg_depth_of_ROI ;*/

					/* sends gray images */
					cv::cvtColor(left_im_color , left_im_gray  , CV_BGR2GRAY);
					cv::cvtColor(right_im_color, right_im_gray , CV_BGR2GRAY);

					// delivers new input , when the process is waiting (not in calculation process)
					localDisp.set_disparity_input(right_im_gray,left_im_gray);  
					
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
						///cout   << " avg_disperity " << avg_disperity  << " avg_depth_of_ROI " << avg_depth_of_ROI << endl;
						////////////

						/* displays */
						myGUI.display_rectified_pair( disperity_struct.imageSize , disperity_struct.rectR , disperity_struct.rectL, 
														disperity_struct.validROI1 , disperity_struct.validROI2 );
						imshow(myGUI.plotWindowsNames[2],  myGUI.plotImages[2]);

						lastDepthImg		= modified_disperity_mat.clone();

						int an=3;	//an=1->kernel of 3
						Mat element = getStructuringElement(MORPH_RECT, Size(an*2+1, an*2+1), Point(an, an) );
						medianBlur	(lastDepthImg,	lastDepthImg,	9);//9//3
						erode		(lastDepthImg , lastDepthImg, element);									
						dilate		(lastDepthImg,	lastDepthImg, element);
						Mat depthMask = lastDepthImg.clone();

						imshow ( "lastDepthImg", lastDepthImg);
						///cvtColor(depthMask	, depthMask , COLOR_BGR2GRAY);

/*
						vector<int> hull; 
						convexHull(depthMask, hull, true);*/

						///int hullcount = (int)hull.size();
						/*Point pt0 = points[hull[hullcount-1]];

						for( i = 0; i < hullcount; i++ )
						{
							Point pt = points[hull[i]];
							line(img, pt0, pt, Scalar(0, 255, 0), 1,LINE_AA);
							pt0 = pt;
						}
*/
						////imshow("hull", img);



						request_water_shed	= true;
						 
					}
					relative_counter	=	0;
				}
			}
			////////////* get Disperity & DEPTH by stereo *///////////////



			Point movementMassCenter, corected_MassCenter;
			// condition by STANDBY, otherwise - only the tracker is in the loop 
			if ( system_state < FOUND_GOOD_TARGET )
			// will change system_state only when (system_state <= FOUND_SOME_MOVEMENT )
			{
				localBackSubs.find_forgnd( left_im_color(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement
				corected_MassCenter = Point(movementMassCenter.x + BckgndSubROI.x,  movementMassCenter.y + BckgndSubROI.y);
				////actually not needed .. makeContours(localBackSubs.get_foreground_mat()); 

				Mat bgnd = localBackSubs.get_the_background_average();  // display is with ..find fgnd
				imshow("BackSubs background average",bgnd); //debugging
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
				imshow("BackSubs bg-current diff",bgndDiff); //debugging

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
					WSH.calculate_the_watershed(lastDepthImg);//    localBackSubs.get_foreground_mat());
					request_water_shed = false;
				}
			}

			if (system_state >= FOUND_GOOD_TARGET)
			{ 
				////////////////////////////////////////////////
				//			make tracking of the 'goodFeatures'
				//			from previous frame to the new one		
				tracker.processImage(left_im_color.clone()  ,system_state);  

				Point2f targetCenter ;
				
				if ( system_state == FOUND_GOOD_TARGET )
					targetCenter = localBackSubs.get_foreground_center() ;
				else
				{
					targetCenter = Point(tracker.TrkErrX_Avg + left_im_color.size().width/2 ,  left_im_color.size().height/2  );
#ifdef COMPILING_ON_ROBOT
					double thrust_percent = depth(targetCenter) ;//, 
					double pix_to_FOV = 1.0/ /*fx=*/ 375 ;
					double angle	  = atan( tracker.TrkErrX_Avg * pix_to_FOV ); // max of 160 will give 160/375 ~ 0.42 // atand(0.42)=23.1deg~0.4rad
					double turn_ratio = 0.9 ; // first angle, then forward. so target stays in FOV center
					hardwareController.Forward(thrust_percent, angle, turn_ratio);
					//Forward(double thrust_percent, double angle, double turn_ratio)  // TODO: add turn_rate_ratio option
#endif
				}
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
		//		warpAffine(left_im_color,orig_warped,invTrans.rowRange(0,2),Size());
		//		imshow("stabilized",orig_warped);
				
				

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
			}
			// move to broadcast from outside. // separate the graphic layer to only when track is on.
			//									otherwise just draw raw images.
			imshow(myGUI.plotWindowsNames[0],	myGUI.plotImages[0] );
			imshow(myGUI.plotWindowsNames[1],	left_im_color);
			 
		}
		//char c = (char)waitKey(loop_delay);
		int c = waitKey(loop_delay);
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