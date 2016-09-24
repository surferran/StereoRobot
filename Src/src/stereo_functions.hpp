/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

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
 
class myLocalDisparity
{
public:
	struct rectification_outputs{
			Mat rectR,rectL;
			Size imageSize ;
			Rect validROI1, validROI2 ;
			long originalyGivenframeCycle;
		};	

	myLocalDisparity();
	~myLocalDisparity();

	void set_disparity_input(Mat inR, Mat inL, long frameCycleCounter);
	bool get_rectified_and_disparity(Mat& disp_output, rectification_outputs& rectification_vars);
	void convert_disperity_value_to_depth(double in_disp, double & out_depth);

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
};

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

myLocalDisparity::myLocalDisparity():
	/* initializing , and executing the thread */
	stereoIm_thrd(&myLocalDisparity::thread_loop_function,this)  
{}

myLocalDisparity :: ~myLocalDisparity()
{
	exit = true;
	stereoIm_thrd.join();
}

/////////////////////////////////////////////////////////////////////////

void myLocalDisparity::thread_loop_function() {

	int		argc;
	char*	argv[11];  
	
	Size imgSize= Size(working_FRAME_WIDTH,	working_FRAME_HIGHT);	// desired resolution for the images, same as in ImageSourceHandler 


	argc = 8;//11;

	argv[3] = "-i";
#ifdef COPMILING_ON_ROBOT
	argv[4] = "/root/RAN/StereoRobot/src/data/intrinsics.yml";
	argv[5] = "-e";
	argv[6] = "/root/RAN/StereoRobot/src/data/extrinsics.yml";
#else
	argv[4] = "C:/Users/Ran_the_User/Documents/GitHub/StereoRobot/Src/src/data/intrinsics.yml";
	argv[5] = "-e";
	argv[6] = "C:/Users/Ran_the_User/Documents/GitHub/StereoRobot/Src/src/data/extrinsics.yml";
#endif
	////outputs:
	//argv[7] = "-o";	argv[8]  = "../data/disp_out.jpg";
	//argv[9] = "-p";	argv[10] = "../data/points_out.yml";
	  argv[7] = "--no-display";	
	//argv[8]  = "";	argv[9] = "";	argv[10] = "";
	stereo_match_and_disparity_init(argc,argv, imgSize); 

	exit						= false;
	calc_disparity_request		= false; 
	calculating_disparity		= false;
	ready_disparity_result		= false;
	relevantframeCycleIndex	= 0;

	while (!exit) {
		mut.lock();

		if (calc_disparity_request)
		{ 
			/* calculates the disparity according to those inputs.
				at the end of the function - sets :
				ready_disparity_result  = true;
				calc_disparity			= false;
			*/
			calc_disparity_request	= false;
			calculating_disparity	= true;
			do_stereo_match( imR, imL, disp_out); // openCV demo as base code
			calculating_disparity	= false;
		}
		
		//TODO: //calcTimeTag = ..  

		mut.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(disparity_loop_dealy));
	}
}

/////////////////////////////////////////////////////////////////////////

int myLocalDisparity::stereo_match_and_disparity_init(int argc, char** argv,  Size img_size)
{  
	if(argc < 3)  
    { 
        return 0;
    } 

	//alg						= STEREO_SGBM;	
	alg						=	 STEREO_BM;
	desired_param_set		=	3;

	SADWindowSize			= 0;
	numberOfDisparities		= 0;
	no_display				= false;
	scale					= 1.f;

	target_image_size		=	img_size;

    for( int i = 1+2; i < argc; i++ )
    {
        //if( argv[i][0] != '-' )
        //{
        //  /*  if( !img1_filename )
        //        img1_filename = argv[i];
        //    else
        //        img2_filename = argv[i];*/
        //}
        //else 
		if( strncmp(argv[i], algorithm_opt, strlen(algorithm_opt)) == 0 )
        {
            char* _alg = argv[i] + strlen(algorithm_opt);
            alg = strcmp(_alg, "bm") == 0   ? STEREO_BM :
                  strcmp(_alg, "sgbm") == 0 ? STEREO_SGBM :
                  strcmp(_alg, "hh") == 0   ? STEREO_HH :
                  strcmp(_alg, "var") == 0  ? STEREO_VAR : -1;
            if( alg < 0 )
            {
                printf("Command-line parameter error: Unknown stereo algorithm\n\n");
                //print_help();
                return -1;
            }
        }
        else if( strncmp(argv[i], maxdisp_opt, strlen(maxdisp_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(maxdisp_opt), "%d", &numberOfDisparities ) != 1 ||
                numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
            {
                printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
                //print_help();
                return -1;
            }
        }
        else if( strncmp(argv[i], blocksize_opt, strlen(blocksize_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(blocksize_opt), "%d", &SADWindowSize ) != 1 ||
                SADWindowSize < 1 || SADWindowSize % 2 != 1 )
            {
                printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
                return -1;
            }
        }
        else if( strncmp(argv[i], scale_opt, strlen(scale_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(scale_opt), "%f", &scale ) != 1 || scale < 0 )
            {
                printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
                return -1;
            }
        }
        else if( strcmp(argv[i], nodisplay_opt) == 0 )
            no_display = true;
        else if( strcmp(argv[i], "-i" ) == 0 )
            intrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-e" ) == 0 )
            extrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-o" ) == 0 )
            disparity_filename = argv[++i];
        else if( strcmp(argv[i], "-p" ) == 0 )
            point_cloud_filename = argv[++i];
        else
        {
            printf("Command-line parameter error: unknown option %d %s\n", i, argv[i]);
            //return -1;
        }
    }
 
    if( (intrinsic_filename != 0) ^ (extrinsic_filename != 0) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename == 0 && point_cloud_filename )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

	if( intrinsic_filename )
	{
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, FileStorage::READ);
		if(!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename);
			return -1;
		}

		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		M1 *= scale;
		M2 *= scale;

		fs.release();

		FileStorage fs2(extrinsic_filename, FileStorage::READ);
		///fs2.open(extrinsic_filename, FileStorage::READ);
		if(!fs2.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
			return -1;
		}

		fs2["R"] >> R;
		fs2["T"] >> T;

		fs2.release();

		/* initialize rectification mapping */
		/* 
			when calibrated and saving matrices - i calibrated 
				Left camera as img1 , 
				Right camera as img2
		*/
		stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	}

	//// set algorithm and parameters :
	if (alg == STEREO_SGBM)
	{ 
		if (desired_param_set==1)
			set_SGBM_params_options_1();
		else
			set_SGBM_params_options_2();
	}
	else if (alg == STEREO_BM)
	{
		switch (desired_param_set)
		{
		case 1:
			set_BM_params_options_1(); break;
		case 2:
			set_BM_params_options_2(); break;
		case 3:
			set_BM_params_options_3(); break;
		default:
			break;
		} 
	}
	else ; //ERROR about alg type


	return 0;
}

//// set parameters according to some recomandations 
void myLocalDisparity::set_BM_params_options_1()
{  

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((target_image_size.width/8) + 15) & -16;
	numberOfDisparities = 112;

	////  bm  params

	bm->setROI1(roi1);
	bm->setROI2(roi2);
	bm->setPreFilterCap(31);
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(15);
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);

	
}

//// set parameters according to some recomandations 
void myLocalDisparity::set_BM_params_options_2()
{  

	set_BM_params_options_1();

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((target_image_size.width/8) + 15) & -16;
	numberOfDisparities = 112;

	//// from ref web site :..
	//SADWindowSize=9;
	//bm->setBlockSize(SADWindowSize) ;
	//bm.state->numberOfDisparities = 112;
	//bm.state->preFilterSize = 5;
	//sbm.state->preFilterCap = 61;
	//sbm.state->minDisparity = -39;
	//sbm.state->textureThreshold = 507;
	//sbm.state->uniquenessRatio = 0;
	//sbm.state->speckleWindowSize = 0;
	//sbm.state->speckleRange = 8;
	//sbm.state->disp12MaxDiff = 1;
	 
	 
}


//// set parameters according to some recomandations 
void myLocalDisparity::set_BM_params_options_3()
{  
	numberOfDisparities	=	12;//8;		//128		//	/8->48 
	numberOfDisparities	=	  ((target_image_size.width/8/4) + 15) & -16;

	bm->setPreFilterSize	( 41 );
	bm->setPreFilterCap		( 31 );
	bm->setBlockSize		( 41 );//SADWindowSize
	bm->setMinDisparity		(-numberOfDisparities/2 );//-64
	bm->setNumDisparities	( numberOfDisparities );
	bm->setTextureThreshold	( 10 );
	bm->setUniquenessRatio	( 15 );

	/*bm->setROI1(roi1);
	bm->setROI2(roi2);
	bm->setBlockSize( > 0 ? SADWindowSize : 9);	
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);*/

}

//// set parameters according to some recomandations 
void myLocalDisparity::set_SGBM_params_options_1()
{  

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((target_image_size.width/8) + 15) & -16;
	numberOfDisparities = 112;

	// sgbm params

	// sgbm is Semi Global Block Matching
	sgbm->setPreFilterCap(63); 
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);

	int cn = 1;//img1.channels();

	sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(-10);//0
	sgbm->setNumDisparities(160);//160//numberOfDisparities/////////////////
	sgbm->setUniquenessRatio(50);//10
	sgbm->setSpeckleWindowSize(10);//100
	sgbm->setSpeckleRange(96);//32
	sgbm->setDisp12MaxDiff(10);//1
	sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

	/////////////////////

}
//// set parameters according to some recomandations 
void myLocalDisparity::set_SGBM_params_options_2()
{  

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((target_image_size.width/8) + 15) & -16;
	numberOfDisparities = 112;

	sgbm->setP1(600);   // 600
	sgbm->setP2(2400); //2400
	sgbm->setMinDisparity(-64);						// -64
	sgbm->setNumDisparities(numberOfDisparities);//192
	sgbm->setUniquenessRatio(1);	//1
	sgbm->setSpeckleWindowSize(150);//150
	sgbm->setSpeckleRange(2);//2
	sgbm->setDisp12MaxDiff(10);//10
	sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);
	
	/*
	sgbm.SADWindowSize = 5;
	sgbm.preFilterCap = 4;
	sgbm.fullDP = false;*/
	

	/////////////////////

}

/////////////////////////////////////////////////////////////////////////

int myLocalDisparity::do_stereo_match(Mat imgR, Mat imgL , Mat& disp8 )
{
	ready_disparity_result  = false;	// runover current unused previous result. if any.
	/* 
		when calibrated and saving matrices - i calibrated 
			Left camera as img1 , 
			Right camera as img2
	*/
	img1 = imgL.clone();
	img2 = imgR.clone(); 

	// scale if image sizes are different then calibrated images resultion
 /*   if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }*/

    Size img_size = img1.size();

    if( intrinsic_filename )
    {
		/* passed to init function
        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);*/

        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;

		rectification_output_vars.imageSize					= img_size;
		rectification_output_vars.rectR						=	img2;
		rectification_output_vars.rectL						=	img1;
		rectification_output_vars.validROI1					=	roi1;
		rectification_output_vars.validROI2					=	roi2;
		rectification_output_vars.originalyGivenframeCycle	=	relevantframeCycleIndex;
				
    }


    Mat		disp;

    int64	t = getTickCount(); 

    if( alg == STEREO_BM ){
		// connversion not needed because already gray and UINT8
		//img1.convertTo(img1, CV_8UC1+CV_BGR2GRAY);
		//img2.convertTo(img2, CV_8UC1+CV_BGR2GRAY);
		
        bm->compute(img1, img2, disp);
		}
    else if( alg == STEREO_SGBM || alg == STEREO_HH )
        sgbm->compute(img1, img2, disp);

    t = getTickCount() - t;
  ////
	printf("STEREO_BM/STEREO_SGBM Time elapsed: %fms\n\n", t*1000/getTickFrequency());

    //	disp = disp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);

    if( !no_display )
    {
        namedWindow("disparity", 0);   imshow("disparity", disp8);
		Mat tmpD ;
			applyColorMap(disp8,tmpD, cv::COLORMAP_JET);
			namedWindow("disparity Heat", 1);   imshow("disparity Heat", tmpD); 
       /// fflush(stdout); 
        printf("\n");
    }
	
    if(disparity_filename)
        imwrite(disparity_filename, disp8);

	{
		Mat xyz_again;
		///http://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
///#ifndef COMPILING_ON_ROBOT
		if (1==2)		
		{
			///Q.at<double>(3,2) = Q.at<double>(3,2)       ;////10.0;
			reprojectImageTo3D(disp, xyz_again, Q, true); 
			Vec3f point_middle = xyz_again.at<Vec3f>(xyz_again.rows/2, xyz_again.cols/2);
			printf("\n\n middle point relative coor. are: %f %f %f \n\n", point_middle.val[0],point_middle.val[1],point_middle.val[2]);
		}
///#endif
	}

	ready_disparity_result  = true;

    return 0;
}


/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void myLocalDisparity::set_disparity_input(Mat inR, Mat inL, long relevantCycleCounter)
{
//	mut.lock();	//?
	if ((!calculating_disparity) && (!ready_disparity_result))	//TODO : check also for used_result?
	{
		imR							= inR.clone();
		imL							= inL.clone();
		relevantframeCycleIndex	= relevantCycleCounter;
		calc_disparity_request	= true;
	}
//	mut.unlock();//?
}

bool myLocalDisparity::get_rectified_and_disparity(Mat& disp_output, rectification_outputs& rectified_vars)
{
///	mut.lock();	//?
	if (ready_disparity_result)	// TODO: verify - not another match in process?
	{
		disp_output					=	disp_out.clone();
		rectified_vars.imageSize	=	rectification_output_vars.imageSize;
		rectified_vars.rectL		=	rectification_output_vars.rectL;
		rectified_vars.rectR		=	rectification_output_vars.rectR;
		rectified_vars.validROI1	=	rectification_output_vars.validROI1;
		rectified_vars.validROI2	=	rectification_output_vars.validROI2;
		rectified_vars.originalyGivenframeCycle	=	rectification_output_vars.originalyGivenframeCycle;
		calcuatedRequestsCounter++;
		ready_disparity_result		= false;
		return true;
	}
	else
		return false;
///	mut.unlock();//?
}


void myLocalDisparity::convert_disperity_value_to_depth(double in_disp, double & out_depth)
{

	// manual settings
	double camera_Base     = 0.06 ; //[m]
	double Focal_lenght    = 375 ;	//[pix]
	double constant_offset = 0;		//[m] 
	double scale_factor    = 67.852222393615605;/// as 1/W 	// homogenic depth (Z/w) to real depth (Z) by *1/w

	//perspectiveTransform?

	/* bf/d */
	if (in_disp > 0)
		out_depth  = constant_offset + (1./in_disp) * camera_Base * Focal_lenght * scale_factor; //[cm] because of the calibration factor
	else
		out_depth = 9990;

}
