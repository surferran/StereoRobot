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

#define MANUAL_TESTINGS		false // true - to allow manual overides to steering (by arrow-keys), and selecting target (by mouse)

/*******************************************************************************************/
/*******************************************************************************************/
/* my  constants and parameters */

// bounds in percent from image size
#define MIN_MOVED_AREA_in_image 33//33//.0//23
#define MAX_MOVED_AREA_in_image 90.0  //95
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
/**********************************/

#include "frameFunctions.h"		// general definitions and functions. that's why it is first to include. 

#include "BackgroundSub.hpp"
#include "RobotController.h"

#include "stereo_calib.h" 
#include "FeatureTracker.hpp"
#include "ImagesSourceHandler.h"
/*********  GUI object  ********/
//#include "myGUI_handler.h"			// cannot include here because will cause loop definitions.
										// it is included in the .cpp file
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
		TRACKING,
		//FOUND_GOOD_TARGET				,
		//TRACKING_GOOD_QUALITY_TARGET	,		// Should show GREEN cross
		//TRACKING_LOW_QUALITY_TARGET		,
		TARGET_IS_LOST			// Should show RED cross	,	after 3 sec will turn to ORANGE (while stopping the robot)
	};

	/* unix key 'int' definitions *///or linux??
	enum ROBOT_KEYS{
	  	UNIX_KEY_UP			= 65362,
	  	UNIX_KEY_DOWN		= 65364,
	  	UNIX_KEY_LEFT		= 65361,
	  	UNIX_KEY_RIGHT		= 65363
	};
	  int	working_FRAME_WIDTH	;
	  int	working_FRAME_HIGHT	;
	  int	working_FRAMES_FPS  ;		// HW can support 15 or 30 fps. practically,with alg.calculations it can be up to 10fps.
	  int	myResizeScaleFactor;		// for resizing the captured image. 1 is nominal case.

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
	SYSTEM_STATUS		system_state ; //= INITIALIZING ;	

	void appInitializations();
	void appMainLoop();

private:


	void check_for_calibration_option();

	bool wait_or_handle_user_input();

	/* inner variables to work with */

	Mat left_im_color ,
		right_im_color;
	Mat left_im_gray ,
		right_im_gray;

	RobotController hardwareController ;

	Mat		target_candidate_features;

	const int		loop_delay = 33 ; //[ms]	// need to fit the file recording value
	char			user_pressing=0;	// just optional.

	int				relative_counter =0;
	 
	Rect		BckgndSubROI;
	Rect		TrackingROI;

	Mat			lastDepthImg,
				depthAvg;
	int			depthAvgNdx = 0;

	bool	got_1st_stable_bkgnd ; 

	Mat		current_disparity = Mat();
	Mat		sum_of_N_disparities = Mat();
	Mat		modified_disperity_mat;
	Scalar	avg_disperity_S;
	double	avg_disperity;
	double	avg_depth_of_ROI	= 0 ,
			effective_depth_measurement = 0;
	double	last_min_depth_of_ROI	= 0 ;

	Point	movementMassCenter, corected_MassCenter;
	Mat		current_mask1,
			current_mask2;
	Mat		bgnd;

	bool	gotNewDispImageToWorkWith = false;

	myCQueue<Mat>		matQueue;
	myCQueue<double>	doubleQueue;					

	Mat featTrackMask_fromBgSubt, 
		featTrackMask_fromDisperity,
		featTrackMask;		//M1 & M2 -> M

	Mat					tmpROI; 
	bool				trackerNotOff;

	int w		,
		h		; 

	int userFwdThrust_percent,
		initialUserFwdThrust_percent;

	int MovementNoTrackCycles,
		MovementNoTrackCycles_Allowed;

};

extern StereoRobotApp myCApp;