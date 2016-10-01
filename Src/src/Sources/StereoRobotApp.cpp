/*
 *  StereoRobotApp.cpp  
 */

#include "..\Headers\StereoRobotApp.hpp"
 
/* class initiation */
StereoRobotApp::StereoRobotApp()
{ 
	working_FRAME_WIDTH		=	320 /1 ;// 640;// 160;
	working_FRAME_HIGHT		=	240 /1 ;// 480;// 120;

	frame_boundary_W_init	=	0;
	frame_boundary			=	0;

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