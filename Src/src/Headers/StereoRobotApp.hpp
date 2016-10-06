/*
 *  StereoRobotApp.hpp  
 */

#pragma once

 /*******************************************************************************************/
 /*******************************************************************************************/
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

using namespace cv;

#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

/*******************************************************************************************/
/*******************************************************************************************/
///extern myGUI_handler myGUI; // thread for images displaying
// maybe use in a different way such as :
//global.h
//extern int myVar;

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

////////////////////////////////


/*********  thread object  ********/
#include "Depth_and_Disparity.hpp"		//set the disparity object (variables and functions)
//Depth_and_Disparity localDisp;
/**********************************/

#include "frameFunctions.h"		// general definitions and functions. that's why it is first to include. 

//StereoRobotApp::SYSTEM_STATUS	system_state = StereoRobotApp::INITIALIZING ;	//TODO: put under the main app OBJ


#include "BackgroundSub.hpp"
#include "RobotController.h"

#include "stereo_calib.h" 
#include "FeatureTracker.hpp"
#include "ImagesSourceHandler.h"
/*********  GUI object  ********/
//#include "myGUI_handler.h"
//extern myGUI_handler		myGUI;

#ifdef COMPILING_ON_ROBOT
										////#include "OdroidC1_handlers/RobotController.h"
#include <unistd.h>
#include "pwm.h"
#endif

/*******************************************************************************************/
/*******************************************************************************************/

class StereoRobotApp
{
public:

	StereoRobotApp();
	~StereoRobotApp();
	
	/* system status to include BackgroundSubstraction & Tracker. */
	enum SYSTEM_STATUS{
		INITIALIZING			=	0 ,		// Should show GRAY cross	 
		STANDBY					=	1 ,		// Should show ORANGE cross
		FOUND_SOME_MOVEMENT				,
		FOUND_GOOD_TARGET				,
		TRACKING_GOOD_QUALITY_TARGET	,		// Should show GREEN cross
		TRACKING_LOW_QUALITY_TARGET		,
		TARGET_IS_LOST			// Should show RED cross	,	after 3 sec will turn to ORANGE (while stopping the robot)
	};

	/* unix key 'int' definitions *///or linux??
	enum ROBOT_KEYS{
	  	UNIX_KEY_UP			= 65362,
	  	UNIX_KEY_DOWN		= 65364,
	  	UNIX_KEY_LEFT		= 65361,
	  	UNIX_KEY_RIGHT		= 65363
	};
	  int	working_FRAME_WIDTH	= 320 /1 ;// 640;// 160;
	  int	working_FRAME_HIGHT	= 240 /1 ;// 480;// 120;
	  int	working_FRAMES_FPS  = 15;		// HW can support 15 or 30 fps. practically,with alg.calculations it can be up to 10fps.

	  int	minDepth_toGoTo;
	  int	maxDepth_toGoTo;

	  int	frame_boundary_W_init;
	  int	frame_boundary;

	struct Operation_flags{
		bool make_stereo_calibration	 ;
		bool calc_background_subs		 ;
		bool show_vid_source_selection	 ; 
										 
		bool show_stereo				 ;
		bool play_on					 ;
		bool save_disparity_img_to_file  ;   
										 
		bool make_camshift				 ;
										 
		bool reset_vid_file_location	 ;
		bool reset_identification		 ;
										 
		bool show_img_hist				;
		bool proces_img_frame			;
		bool draw_middle_x				;
	};
	
	Operation_flags	op_flags; 

	/* ******** application objects ******** */
	/*Depth_and_Disparity localDisp;*/
	SYSTEM_STATUS		system_state ; //= INITIALIZING ;	

	void appInitializations();
	void appMainLoop();

private:

	bool wait_or_handle_user_input();

	/* inner variables to work with */

	Mat left_im_color ,
		right_im_color;
	Mat left_im_gray ,
		right_im_gray;

	RobotController hardwareController ;

	Mat		target_candidate_features;

	///bool request_water_shed = false;

	const int		loop_delay = 33 ; //[ms]	// need to fit the file recording value
	char			user_pressing=0;	// just optional.

	int				relative_counter =0;

	///*  initiating images capturing  */
	//ImagesSourceHandler myStereoCams; // thread for images capturing
	//								  /*  ***************************  */

									  /*  initiating createBackgroundSubtractorMOG2   */
	/*BackSubs	localBackSubs ;*/
	/*  *****************************************   */
	/*  initiating target   */
	//Target	first_target ;
	/*  *****************************************   */
	//const int target_lost_timeout_counter  = 2* loop_delay ; // [~sec]//counter to simulate delay of about 2 sec. (depend on loop inner delay)
	//int		  target_lost_time_counter = 0 ;   // stopper to timeout
	
	/*Tracker		tracker;*/
	Rect		BckgndSubROI;
	Rect		TrackingROI;

	Mat			lastDepthImg,
				depthAvg;
	int			depthAvgNdx = 0;

	bool	got_1st_stable_bkgnd ;
/*
	Depth_and_Disparity::rectification_outputs disperity_struct;*/
	Mat		current_disparity = Mat();
	Mat		sum_of_N_disparities = Mat();
	Mat		modified_disperity_mat;
	Scalar	avg_disperity_S;
	double	avg_disperity;
	double	avg_depth_of_ROI	= 0 ;
	double	last_min_depth_of_ROI	= 0 ;

	Point	movementMassCenter, corected_MassCenter;
	Mat		current_mask1,
			current_mask2;
	Mat		bgnd;

	bool	gotNewDispImageToWorkWith = false;

	myCQueue<Mat>		matQueue;
	myCQueue<double>	doubleQueue;					

	Mat featTrackMask1, 
		featTrackMask2,
		featTrackMask;		//M1 & M2 -> M

	Mat					tmpROI;/*
	Target::TargetState tmpTargStat;*/
	bool				trackerNotOff;

	int w		,
		h		;
		//waitSec ;


};

extern StereoRobotApp myCApp;