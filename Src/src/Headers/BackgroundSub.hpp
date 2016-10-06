// file : BackgroundSub.hpp

#pragma once

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <stdio.h>

//#include "opencv2/opencv.hpp"
//#include "GUIFunctions.h"		// for using _intToString
//#include "myGUI_handler.h"
#include "StereoRobotApp.hpp"

using namespace cv;

//extern myGUI_handler	myGUI ;
//extern StereoRobotApp::SYSTEM_STATUS	system_state ;
///extern StereoRobotApp	myCApp;

/*************************************************************************************/
/******************************    Header section    *********************************/
/*************************************************************************************/
class BackSubs
{
public:

	BackSubs();

	enum BgSubt_STATE {
		INITIALIZING	=	0,
		STANDING_BY		=	1,
		FOUND_MOVEMENT	=	2
	} BgSubt_Status ;

	int		show_forgnd_and_bgnd_init(int a); 

	void	find_forgnd(Mat frame, Point *movementMassCenter);

	Mat		get_foreground_mat()		{ return foreground.clone() ; } ;
	Rect	get_foreground_boundRect()	{ return boundRect			; } ;
	Point	get_foreground_center()		{ return MassCenter			; } ;
	Mat		get_the_background_average(){ mog->getBackgroundImage(backgroundAvg) ; return backgroundAvg; } ;

	 
private:
	
	///int show_more_details(Mat frame) ;
	int doMYbsManipulation( Mat & mask , Point *movementMassCenter);
	 
	String					StatusText	= "NAN";

	int						stable_bkgnd_phase = 0; 

	int						fps;
	Mat						frame, foreground, image;	// inner vars for class functions.
	Mat						middle_tmp_frame;			//	
	Mat						backgroundAvg ;

	/* for show_more_details() */
	vector<vector<Point> >	contours;
	vector<vector<Point> >	selected_shapes_contours;
	int						area;
	///Rect					rect;
    
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
	const int				BackSubs_History		= 300;///60;//120;			//by example: 300 ,32, true
	const double			BackSubs_Threshould		= 32;///16.0;	
	const bool				BackSubs_DetectShadows	= false;   // only for better run-time performance
	const double			BackSubs_LearningRate	= -1.0;  //~-0.5~-0.7? -1

	
};
/*************************************************************************************/
/******************************end of Header section *********************************/
/*************************************************************************************/
