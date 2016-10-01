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

};


extern StereoRobotApp myCApp;