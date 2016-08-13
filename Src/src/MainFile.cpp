//// 3D stero robot (RoboDog)

// TODO : 
//  26/11/15 -  set working_consts by UI, esspecialy choosing REC or PLAYBACK
//				set correct recordings rate		
//				do calibration !!
// DONE :
//	14/11/15 - performance issue..  -> 25/11/15 - smaller resultion to reduce bus trafic !
//  14/02/16 - stereo calibration seems to work finally. smaller resultution captures are bad for calibration.

#define RUN_ON_LAPTOP__MONO true   // true state is not working well. and actually not necessary.

 /* my  constants and parameters */
#define LEFT_CAMERA_INDEX		2		// depends on platform. 0 index is the default camera.
#define RIGHT_CAMERA_INDEX		1
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
#include "utilFunctions.h"		// utility functions for the application.

#include "BackgroundSub.hpp"

#include "GUIFunctions.h"

// also : https://en.wikipedia.org/wiki/Image_moment#Examples_2
#include "camshiftdemo.cpp"

#include "./SBM_Sample.cpp"		// externals/

#include "stereo_calib.h" 

#include "myTracker.cpp"

//#include "showManyImages.cpp"
void cvShowManyImages(char* title, int nArgs, ...) ;

////when need to eliminate the consule that is opened in parallel 
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

void specific_match()
{
	char*	argv2[2];  //6
	//// TODO: put in a loop to get those from cameras
	argv2[0] = "../run_inputs/stereo_calibration_images/shot_1_031.jpg";  // left image
	argv2[1] = "../run_inputs/stereo_calibration_images/shot_0_031.jpg";  // right image
	Mat imgL = imread(argv2[1], -1);
	Mat imgR = imread(argv2[0], -1);

	IplImage	*im_mat1 = cvCloneImage(&(IplImage)imgL),
				*im_mat2 = cvCloneImage(&(IplImage)imgR);
	cvShowManyImages("image couple" , 4 , im_mat1, im_mat2, im_mat1, im_mat2 );
	cvShowManyImages("image couple2" , 2 , im_mat1, im_mat2 );
	waitKey(0);
	return;

	Mat outM;

	///////////////////////////////

	Mat imgLeft		= imread( argv2[0], IMREAD_GRAYSCALE );
	Mat imgRight	= imread( argv2[1], IMREAD_GRAYSCALE );
	 
	main_SBM(imgLeft,imgRight, outM);

	imshow("right",imgR);
	imshow("left" ,imgL);
	imshow("disp out",outM);
	
	waitKey(0);

	destroyAllWindows();

	return;
	///////////////////////////////
/*
	do_stereo_disp_init();
	do_stereo_disp(imgR,imgL,outM);

	imshow("right",imgR);
	imshow("left",imgL);
	imshow("disp out",outM);

	waitKey(0);

	destroyAllWindows();*/
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////					  main	   			  //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) 
{
	///		specific_match();  // testing for sending right parameters, and images order

	/* variables */

	thisStereo.input_source = LIVE_CAM;
		
	//show_buttons_gui();  
	op_flags.show_stereo=true;	// initialize and conduct stereo algo imidiatly when running.

	string	base_out_file_path	= "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/out";
	string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".*";	
	int		stream_frame_index	= 0;
	char	rec_file_name[150]  = "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/in/VID_3D_scenario/output_1.avi";
	VideoCapture vid			= VideoCapture(rec_file_name);	
	Mat		target_candidate_features;
	Mat		tracked_target_image;

	bool first_setup = true;

	int		loop_delay = 33 ; //[ms]	// need to fit the file recording value
	char	user_pressing=0;	// just optional.

	int relative_counter =0;
	VideoCapture vidR,vidL;

	const int target_lost_timeout      = 500 ; // counter to simulate delay of about 2 sec. (depend on loop inner delay)
	int		  target_lost_time_counter = 0 ;   // stopper to timeout

	if( !vid.isOpened() )
		return -1;

	plotWindowsNames[0] = "win1 - right stereo image";
	plotWindowsNames[1] = "win2 - left stereo image"; 
		// TODO: added frame of copmosed stiched image ? can take relative very long process.
			// anyway can add as ref. with other low-prio thred
		//  it is actually part of the disparity process.
	plotWindowsNames[2] = "win3 - calculated disparity";
	plotWindowsNames[3] = "win4 - background substruction output";
	plotWindowsNames[4] = "win5 - tracked object";

	BackSubs	localBackSubs ;

	Tracker		tracker;
	Rect		BckgndSubROI;
	Rect		TrackingROI;

	bool got_1st_stable_bkgnd = false;

	/* end of variables */

	////////////// 1st entrance only ///////////
	if (first_setup) {
		first_setup = false;
		////check of sources already active
		//vidR			= VideoCapture(1);	
		//vidL			= VideoCapture(2);	
		int w=320,	h=240 , waitSec = 5;
		if (RUN_ON_LAPTOP__MONO)
		{
			vidL.open(0);	
			vidL.set(CAP_PROP_FRAME_WIDTH, w);	vidL.set(CAP_PROP_FRAME_HEIGHT, h);
		}
		else
		{
			vidR.open(RIGHT_CAMERA_INDEX);	
			vidL.open(LEFT_CAMERA_INDEX);

			if(!vidR.isOpened()){ cout << "Cannot open right camera" << endl; return -1;}
			if(!vidL.isOpened()){ cout << "Cannot open left camera" << endl;  return -1;}

			vidR.set(CAP_PROP_FRAME_WIDTH, w);	vidR.set(CAP_PROP_FRAME_HEIGHT, h);
			vidL.set(CAP_PROP_FRAME_WIDTH, w);	vidL.set(CAP_PROP_FRAME_HEIGHT, h);
			cout <<" waiting "<<waitSec<<" sec to initialze cameras";
			cvWaitKey(waitSec*1000); // initial delay for init
			cout <<" .. continuing ";
		}

		localBackSubs.show_forgnd_and_bgnd_init(vidL); //with Left cam  


														/* clear points that are out of my desired ROI (center of image) */
														//TODO:make 20 h , 30 w /// sizes are for after resize
														//CV_EXPORTS_W void rectangle(InputOutputArray img, Point pt1, Point pt2,
														//	const Scalar& color, int thickness = 1,
														//	int lineType = LINE_8, int shift = 0); 
		Point	TopLeft(frame_boundary, frame_boundary); 
		Point	LowRight(w - frame_boundary , h - frame_boundary);
		TrackingROI = Rect(TopLeft, LowRight ); 

		//	ROI for background substraction is narrower then the one for the tracker
		TopLeft		 = Point(frame_boundary_W_init, frame_boundary); 
		LowRight	 = Point(w - frame_boundary_W_init , h - frame_boundary);
		BckgndSubROI = Rect(TopLeft, LowRight ); 
	}
	////////////// end of 1st entrance only ///////////

	// prepare display windows locations
	
	//drawMatches
	

	if (!RUN_ON_LAPTOP__MONO)
		do_stereo_disp_init();


	while (1)		// TODO: add delay for the loop. about 10mS
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
			relative_counter++;

			////////////// capture images ///////////
			if (RUN_ON_LAPTOP__MONO){
				vidL >> plotImages[1];
				plotImages[0] = plotImages[1];  // questionable ..
			}
			else
			{
				vidR >> plotImages[0];
				vidL >> plotImages[1];
			}
			////////////// end of capture images ///////////
			 
			Mat left_cam = plotImages[1].clone();   

			Point movementMassCenter;
			// condition by STANDBY, otherwise - only the tracker is in the loop 
			if ( system_state < FOUND_GOOD_TARGET )
			// will change system_state only when (system_state <= FOUND_SOME_MOVEMENT )
				localBackSubs.find_forgnd( left_cam(BckgndSubROI) , &movementMassCenter ) ; //// synthesize target by movement

			////////////* get DEPTH by Stereo *///////////////
			if (!RUN_ON_LAPTOP__MONO){
				// calc disparity every 2 frame
				if (relative_counter>1) //10  
				{
					cv::cvtColor(plotImages[0+1], plotImages[0+1], CV_BGR2GRAY);
					cv::cvtColor(plotImages[0*1], plotImages[0*1], CV_BGR2GRAY);

					do_stereo_disp(plotImages[0+1],plotImages[1*0], plotImages[2]);  // plotImages[2] is the disparity output
					///disp relevant disperity values. 
					// for image blobs or average areas. use superpixel segmentation??
					// get disp average for the target feature poitns area.

					//main_SBM(plotImages[0+1],plotImages[1*0], plotImages[2]); 
					imshow(plotWindowsNames[2],  plotImages[2]);
					relative_counter	=	0;
				}
			}
			////////////* get DEPTH by stereo *///////////////


			////////////////////////////////////////////////
			//			tracking part (by 'goodFeatures')
			if ( system_state == FOUND_GOOD_TARGET )
			{	// want to init and lock the tracker
				// get the feature points of the target from the BackgroundSubs ROI
				Mat candidate_features;
				target_candidate_features = localBackSubs.get_foreground_mat();//*left_cam ;	//element-wise multiplication
				cvtColor( target_candidate_features , candidate_features , CV_GRAY2BGR);
				//candidate_Features = candidate_Features.mul(left_cam(BckgndSubROI)  ) ; 
				//candidate_Features = left_cam(BckgndSubROI)  . mul(candidate_Features);
				left_cam(BckgndSubROI).copyTo(tracked_target_image, candidate_features);
				imshow("tracked Target candidates", tracked_target_image) ; // show 4 debug  only
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
					tracker.processImage(left_cam (TrackingROI), left_cam (TrackingROI) ,system_state);  
					 
				if (tracker.TrackPercent > 25)
					system_state = TRACKING_LOW_QUALITY_TARGET;
				if (tracker.TrackPercent > 80)
					system_state = TRACKING_GOOD_QUALITY_TARGET;
				else 
					if ( ( (tracker.TrackPercent > 95) && (target_lost_time_counter < target_lost_timeout) ) 
						 || (target_lost_time_counter > 0) )
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
				left_cam	= plotImages[1].clone();   // for some additional display layer

				if (op_flags.draw_middle_x)
				{
					//on the right image
					//add_Cross_to_Image(left_cam.size[1]/2, left_cam.size[0]/2, false, system_state , left_cam); // 120h,160w , with no coor. label
					add_Cross_to_Image(movementMassCenter.x + BckgndSubROI.x,  movementMassCenter.y + BckgndSubROI.y , 
											false, system_state , left_cam); // 120h,160w , with no coor. label
					
				}
				String StatusText = _sysStatToString(system_state );
				putText(left_cam, StatusText, Point(15, 15), FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 255), 1);

				imshow(plotWindowsNames[0],	plotImages[0] );
				imshow(plotWindowsNames[1],	left_cam);
				////////////* end of graphics section *///////////////
			}
			 
		}

		if (op_flags.reset_vid_file_location) // TODO: verify source is file and not camera.
		{
			vid.set(CAP_PROP_POS_FRAMES,0);	// go back to first frame
			op_flags.reset_identification	 = true;
			op_flags.reset_vid_file_location = false;
		}

		if (op_flags.reset_identification){
			reset_camshift_vars();			// a funciton to reset the camshift main variables. 
			op_flags.reset_identification	= false;
		}

		if (op_flags.play_on){

			vid >> plotImages[0];
			if (plotImages[0].empty())
				op_flags.play_on = false;
			else
			{					
				imshow(plotWindowsNames[0], plotImages[0]); 
				if (op_flags.make_camshift)
					do_camshift_on_Single_current_frame(&plotImages[0]); 

				if (op_flags.show_stereo) /// TODO!!
				{
					imshow(plotWindowsNames[1], plotImages[0]);
					imshow(plotWindowsNames[2], plotImages[1]);
					///plot_images[2] = calc_disparity();
					/*imshow(plotWindowsNames[3], plotImages[2]);*/

				}
				if (op_flags.show_vid_source_selection)
				{
					
				}
			}


		}
		
		if (!check_user_input(&loop_delay, &user_pressing))
			break;
	}
	
	return 0;

}