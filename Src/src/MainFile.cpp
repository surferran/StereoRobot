//// 3D stero robot (RoboDog)

// TODO : 
//  26/11/15 -  set working_consts by UI, esspecialy choosing REC or PLAYBACK
//				set correct recordings rate		
//				do calibration !!
// DONE :
//	14/11/15 - performance issue..  -> 25/11/15 - smaller resultion to reduce bus trafic !
//  14/02/16 - stereo calibration seems to work finally. smaller resultution captures are bad for calibration.

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

#include "GUI_functions.h"

// also : https://en.wikipedia.org/wiki/Image_moment#Examples_2
#include "camshiftdemo.cpp"

#include "./SBM_Sample.cpp"		// externals/

#include "stereo_calib.h" 

#include "myTracker.cpp"

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

int main(int argc, char** argv) 
{
	//	specific_match();  // testing for sending right parameters, and images order

	thisStereo.input_source = LIVE_CAM;
		
	show_buttons_gui();  

	string	base_out_file_path	= "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/out";
	string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".*";	
	int		stream_frame_index	= 0;
	char	rec_file_name[150]  = "C:/Users/Ran_the_User/Documents/Technion_Studies/IP_STUFF/video_4_testing/in/VID_3D_scenario/output_1.avi";
	VideoCapture vid			= VideoCapture(rec_file_name);	

	bool first_setup = true;

	int		loop_delay = 33 ; //[ms]	// need to fit the file recording value
	char	user_pressing=0;	// just optional.

	int relative_counter =0;
	VideoCapture vidR,vidL;

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
			relative_counter++;

			if (first_setup) {
				////check of sources already active
				//vidR			= VideoCapture(1);	
				//vidL			= VideoCapture(2);	

				vidR.open(RIGHT_CAMERA_INDEX);	
				vidL.open(LEFT_CAMERA_INDEX);
				int w=320,	h=240 , waitSec = 5;
				vidR.set(CAP_PROP_FRAME_WIDTH, w);	vidR.set(CAP_PROP_FRAME_HEIGHT, h);
				vidL.set(CAP_PROP_FRAME_WIDTH, w);	vidL.set(CAP_PROP_FRAME_HEIGHT, h);
				first_setup = false;
				cout <<" waiting "<<waitSec<<" sec to initialze cameras";
				cvWaitKey(waitSec*1000); // initial delay for init
				cout <<" .. continuing ";

				localBackSubs.show_forgnd_and_bgnd_init(LEFT_CAMERA_INDEX); //with Left cam
			}

			vidR >> plotImages[0];
			vidL >> plotImages[1];
			
			// add check for empty
			Mat left_cam = plotImages[1].clone();   // for some additional display layer
			if (op_flags.draw_middle_x)
			{
				//on the right image
				add_Cross_to_Image(left_cam.size[1]/2, left_cam.size[0]/2, false, localBackSubs.BackSubs_State , left_cam); // 120h,160w , with no coor. label
			}
			imshow(plotWindowsNames[0],	plotImages[0] );
			imshow(plotWindowsNames[1],	left_cam);
			
			left_cam = plotImages[1].clone();		// copy again, without the added UI graphics

			////////
			localBackSubs.show_forgnd( left_cam ) ;////
			////////

			if (relative_counter>2) //10
			{
				cv::cvtColor(plotImages[0+1], plotImages[0+1], CV_BGR2GRAY);
				cv::cvtColor(plotImages[0*1], plotImages[0*1], CV_BGR2GRAY);

				do_stereo_disp(plotImages[0+1],plotImages[1*0], plotImages[2]);  // plotImages[2] is the disparity output

				//main_SBM(plotImages[0+1],plotImages[1*0], plotImages[2]); 
				imshow(plotWindowsNames[2],  plotImages[2]);
				relative_counter	=	0;
			}

			////////////////////////////////////////////////
			//			tracking part (by 'goodFeatures')

			/* clear points that are out of my desired ROI (center of image) */
			int frame_boundary = 30;  //TODO:make 20 h , 30 w /// sizes are for after resize
									  //CV_EXPORTS_W void rectangle(InputOutputArray img, Point pt1, Point pt2,
									  //	const Scalar& color, int thickness = 1,
									  //	int lineType = LINE_8, int shift = 0);
			Point	TopLeft(frame_boundary, frame_boundary);
			Point	LowRight(left_cam.size().width - frame_boundary , left_cam.size().height - frame_boundary);
			Rect	myGeneralROI = Rect(TopLeft, LowRight ); 

			// add ROI update relevant to BkgSubs points/area only

			////////////////////////////////////////////////
			//			make tracking of the 'goodFeatures'
			//			from previous frame to the new one		
			tracker.processImage(left_cam (myGeneralROI) );   // myROI

			//// check direction change from the previous tracked poits center. mark an arrow .
			//////////////////////////////////////////////////
			////	calculate translation from previous to current.
			////	display modified current 
			Mat invTrans = tracker.rigidTransform.inv(DECOMP_SVD);
			Mat orig_warped;
			warpAffine(left_cam,orig_warped,invTrans.rowRange(0,2),Size());
			imshow("stabilized",orig_warped);


			//op_flags.show_stereo = false;
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