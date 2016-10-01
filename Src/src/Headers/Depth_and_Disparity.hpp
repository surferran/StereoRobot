/*
 *  Depth_and_Disparity.hpp  
 */

#pragma once

#include "..\Headers\StereoRobotApp.hpp"

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

#include "myGUI_handler.h"
extern myGUI_handler myGUI; // thread for images displaying
 
class Depth_and_Disparity
{
public:
	struct rectification_outputs{
			Mat rectR,rectL;
			Size imageSize ;
			Rect validROI1, validROI2 ;
			long originalyGivenframeCycle;
		};	

	Depth_and_Disparity();
	~Depth_and_Disparity();

	bool calc_disperity(int desiredPhase , Mat in_left, Mat in_right, Mat *disperity_out, double *avg_depth);

	void set_disparity_input(Mat inR, Mat inL, long frameCycleCounter);
	bool get_rectified_and_disparity(Mat& disp_output, rectification_outputs& rectification_vars);
	void convert_disperity_value_to_depth(double in_disp, double & out_depth);
	void convert_disperity_Mat_to_depth(Mat in_disp, Mat & out_depth);

	int calcDispEveryNcycles = 1;//3;

	int calcuatedRequestsCounter = 0;

private: 
	int stereo_match_and_disparity_init(int argc, char** argv,  Size img_size);
	int do_stereo_match( Mat imgR, Mat imgL , Mat& disp8 );	// ..and disparity calculation

	void set_BM_params_options_1();
	void set_BM_params_options_2();
	void set_BM_params_options_3();
	void set_SGBM_params_options_1();
	void set_SGBM_params_options_2();
	

	/* in , out matrices */
	Mat		imR, imL, 
			disp_out;
	long	relevantframeCycleIndex ;

	rectification_outputs rectification_output_vars;
	
	/* algorithm variables */
    const char* algorithm_opt = "--algorithm=";
    const char* maxdisp_opt   = "--max-disparity=";
    const char* blocksize_opt = "--blocksize=";
    const char* nodisplay_opt = "--no-display";
    const char* scale_opt	  = "--scale=";

    const char* intrinsic_filename		= 0;
    const char* extrinsic_filename		= 0;
    const char* disparity_filename		= 0;
    const char* point_cloud_filename	= 0;

    enum {	STEREO_BM=0, 
			STEREO_SGBM=1, 
			STEREO_HH=2, 
			STEREO_VAR=3 
		 };
    int		alg					= STEREO_SGBM;
	int		desired_param_set	= 1;		//1,2,3
    int		SADWindowSize		= 0, 
			numberOfDisparities = 0;
	Size	target_image_size	;
    bool	no_display		= false;
    float	scale			= 1.f;

    Ptr<StereoBM>	bm		= StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm	= StereoSGBM::create(0,16,3);

	/* calibration parameters */
	Mat M1, D1, M2, D2;
	Mat R, T, R1, P1, R2, P2;
	Mat Q;

	/* rectification parameters */
	Mat map11, map12, map21, map22;
	Rect roi1, roi2;

	Mat img1r, img2r;
	Mat img1 ;
	Mat img2 ;

	/* variables for thread handling */
	const int		disparity_loop_dealy = 33*1; // [mS] // do only every 3 cycles of images capturing
	std::mutex		mut;
	std::thread		stereoIm_thrd;
	//atomic<bool>
		bool exit; 
	//atomic<bool>
		bool calc_disparity_request,	//user requset flag
			 calculating_disparity;		// busy flag sign
	//atomic<bool>
		bool ready_disparity_result; 

	void thread_loop_function();

	Mat						last_result_of_disparity,
							filtered_disparity;
	rectification_outputs	last_result_of_disparity_struct;
};
