// file : BackgroundSub.hpp

#pragma once

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <stdio.h>

#include "StereoRobotApp.hpp"

using namespace cv;

/*************************************************************************************/
/******************************    Header section    *********************************/
/*************************************************************************************/
class BackSubs
{
public:

	BackSubs();

	enum BgSubt_STATE {
		BG_INITIALIZING	=	0,
		BG_STANDING_BY		=	1,
		BG_FOUND_MOVEMENT	=	2,
		BG_RECOVER_FROM_LOST = 3
	} BgSubt_Status ;

	int		show_forgnd_and_bgnd_init(int fps, bool lost); 

	void	find_forgnd(Mat frame, Point *movementMassCenter);

	Mat		get_foreground_mat()		{ return foreground.clone() ; } ;
	Rect	get_foreground_boundRect()	{ return boundRect			; } ;
	Point	get_foreground_center()		{ return MassCenter			; } ;
	Mat		get_the_background_average(){ mog->getBackgroundImage(backgroundAvg) ; return backgroundAvg; } ;

private:
	
	int checkBgSubt_MaskResult( Mat & mask , Point *movementMassCenter);
	 
	String					StatusText	;

	int						stable_bkgnd_phase ; 

	int						fps;
	Mat						frame, foreground, image;	// inner vars for class functions.
	Mat						middle_tmp_frame;			//	
	Mat						backgroundAvg ;
	int						frame_counter;
	int						init_frames_number;
	int						cycles_number_after_lost;

	/* for show_more_details() */
	vector<vector<Point> >	contours;
	vector<vector<Point> >	selected_shapes_contours;
	int						area;
    
	cv::Ptr<BackgroundSubtractorMOG2> mog ;
	Scalar					random_color;
	int						loopWait = 0;
	int						found_contours_num ;
 
	/* foreground movement features */
	Point	MassCenter ;
	double	rCircle;			// estimated rounding circle for the object area
	Rect	boundRect;
	double	theta ;				// estimated oriantation of bounding box. though not well feature

	/* initializing parameters for the Background_Subtractor algorithm */
	// history - higher is longet 'study'
	// threshould - higher is more restricting (better sureness about not being background)
	// learnRate - (-1) is automatic, 
	const int				BackSubs_History		= 120;////300;///60;//120;			//by example: 300 ,32, true
	const double			BackSubs_Threshould		= 40;///32;///16.0;	
	const bool				BackSubs_DetectShadows	= false;   // only for better run-time performance
	const double			BackSubs_LearningRate	= -1.0;  //~-0.5~-0.7? -1

	//Target BgSubt_TargetMask;
};
/*************************************************************************************/
/******************************end of Header section *********************************/
/*************************************************************************************/
