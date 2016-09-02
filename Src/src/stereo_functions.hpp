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

static void print_helpM()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
           "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
}

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

class myLocalDisparity
{
public:
	struct rect_display_vars{
			Mat rectR,rectL;
			Size imageSize ;
			Rect validROI1, validROI2 ;
		};	

	myLocalDisparity();
	~myLocalDisparity();

	void set_disparity_input(Mat inR, Mat inL);
	bool get_rectified_and_disparity(Mat& disp_output, rect_display_vars& display_vars);

private:
	int do_stereo_match_init(int argc, char** argv);
	int do_stereo_match( Mat imgR, Mat imgL , Mat& disp8 );	// ..and disparity calculation

	/* in , out matrices */
	Mat imR, imL, 
		disp_out;

	rect_display_vars rectified_display_vars;
	
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
    int		alg				= STEREO_SGBM;
    int		SADWindowSize	= 0, numberOfDisparities = 0;
    bool	no_display		= false;
    float	scale			= 1.f;

    Ptr<StereoBM>	bm		= StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm	= StereoSGBM::create(0,16,3);

	Mat M1, D1, M2, D2;
	Mat R, T, R1, P1, R2, P2;

	Mat map11, map12, map21, map22;
	Mat img1r, img2r;
	Mat img1 ;
	Mat img2 ;
	Rect roi1, roi2;
	Mat Q;

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

	void myLocalDisparity::thread_loop_function();
};
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


void myLocalDisparity::thread_loop_function() {

	int		argc;
	char*	argv[11];  //6

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
	do_stereo_match_init (argc,argv); 

	exit					= false;
	calc_disparity_request	= false; 
	calculating_disparity   = false;
	ready_disparity_result	= false;

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

int myLocalDisparity::do_stereo_match_init(int argc, char** argv)
{  
	if(argc < 3)  
    {
        print_helpM();
        return 0;
    } 

	alg				= STEREO_SGBM;
	//alg				= STEREO_BM;
	SADWindowSize	= 0;
	numberOfDisparities = 0;
	  no_display		= false;
	scale			= 1.f;

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

	}

	return 0;
}

int myLocalDisparity::do_stereo_match(Mat imgR, Mat imgL , Mat& disp8 )
{
	ready_disparity_result  = false;	// runover current unused previous result. if any.

	img1 = imgR.clone();
	img2 = imgL.clone();

    if (img1.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (img2.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }

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
        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;

		rectified_display_vars.imageSize = img_size;
		rectified_display_vars.rectL	=	img2;
		rectified_display_vars.rectR	=	img1;
		rectified_display_vars.validROI1=	roi1;
		rectified_display_vars.validROI2=	roi2;

		///void myGUI_handler::display_rectified_pair(Size imageSize , Mat Rimg, Mat Limg, Rect validROI1, Rect validROI2 )
		///myGUI.display_rectified_pair( img_size, img1, img2, roi1, roi2);
		
    }

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
	numberOfDisparities = 112;

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

	// from ref web site :..
	/*
	sbm.state->SADWindowSize = 9;
	sbm.state->numberOfDisparities = 112;
	sbm.state->preFilterSize = 5;
	sbm.state->preFilterCap = 61;
	sbm.state->minDisparity = -39;
	sbm.state->textureThreshold = 507;
	sbm.state->uniquenessRatio = 0;
	sbm.state->speckleWindowSize = 0;
	sbm.state->speckleRange = 8;
	sbm.state->disp12MaxDiff = 1;
	*/
	//

	// sgbm is Semi Global Block Matching
    sgbm->setPreFilterCap(63); 
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(-10);//0
    sgbm->setNumDisparities(160);//160//numberOfDisparities/////////////////
    sgbm->setUniquenessRatio(50);//10
    sgbm->setSpeckleWindowSize(10);//100
    sgbm->setSpeckleRange(96);//32
    sgbm->setDisp12MaxDiff(10);//1
    sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

	// from : http://www.jayrambhia.com/blog/disparity-maps/


	//sgbm->setP1(600);   // 600
	//sgbm->setP2(2400); //2400
	//sgbm->setMinDisparity(-64);						// -64
	//sgbm->setNumDisparities(numberOfDisparities);//192
	//sgbm->setUniquenessRatio(1);	//1
	//sgbm->setSpeckleWindowSize(150);//150
	//sgbm->setSpeckleRange(2);//2
	//sgbm->setDisp12MaxDiff(10);//10
	//sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);
/*
	sgbm.SADWindowSize = 5;
	sgbm.preFilterCap = 4;
	sgbm.fullDP = false;
*/

    Mat disp;//, disp8;
    Mat img1p, img2p;//, dispp;
    copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
	//alg=STEREO_BM;//////RAN
    if( alg == STEREO_BM ){
		img1.convertTo(img1, CV_8UC1+CV_BGR2GRAY);
		img2.convertTo(img2, CV_8UC1+CV_BGR2GRAY);
		
        bm->compute(img1, img2, disp);
		}
    else if( alg == STEREO_SGBM || alg == STEREO_HH )
        sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //	disp = disp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);
    if( !no_display )
    {
       // namedWindow("left", 1);        imshow("left", img1);
       // namedWindow("right", 1);       imshow("right", img2);
        namedWindow("disparity", 0);   imshow("disparity", disp8);
       // printf("press any key to continue...");
        fflush(stdout);
       // waitKey();
        printf("\n");
    }

    if(disparity_filename)
        imwrite(disparity_filename, disp8);

	{
		Mat xyz_again;
		///http://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
#ifndef COMPILING_ON_ROBOT
		if (1==2)		
		{
			Q.at<double>(3,2) = Q.at<double>(3,2)       ;////10.0;
			reprojectImageTo3D(disp, xyz_again, Q, true); 
			Vec3f point_middle = xyz_again.at<Vec3f>(xyz_again.rows/2, xyz_again.cols/2);
			printf("\n\n middle point relative coor. are: %f %f %f \n\n", point_middle.val[0],point_middle.val[1],point_middle.val[2]);
		}
#endif
	}

    if(point_cloud_filename)
    {
		// save to file with low priority. or frequencty.  need to estimate self position
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename, xyz);
        printf("\n");

		// print/display distance to center point or area of the image(left one)
		//TODO:
    }

	ready_disparity_result  = true;
    return 0;
}


void myLocalDisparity::set_disparity_input(Mat inR, Mat inL)
{
//	mut.lock();	//?
	if ((!calculating_disparity) && (!ready_disparity_result))	//TODO : check also for used_result?
	{
		imR						= inR.clone();
		imL						= inL.clone();
		calc_disparity_request	= true;
	}
//	mut.unlock();//?
}

bool myLocalDisparity::get_rectified_and_disparity(Mat& disp_output, rect_display_vars& display_vars)
{
///	mut.lock();	//?
	if (ready_disparity_result)	// TODO: verify - not another match in process?
	{
		disp_output  = disp_out;
		display_vars.imageSize =  rectified_display_vars.imageSize;
		display_vars.rectL	=	rectified_display_vars.rectL;
		display_vars.rectR	=	rectified_display_vars.rectR;
		display_vars.validROI1=	rectified_display_vars.validROI1;
		display_vars.validROI2=	rectified_display_vars.validROI2;

		ready_disparity_result = false;
		return true;
	}
	else
		return false;
///	mut.unlock();//?
}


///myLocalDisparity localDisp;


//void do_stereo_disp_init(){
//
//	int		argc;
//	char*	argv[11];  //6
//
//	argc = 8;//11;
//
//	argv[3] = "-i";
//#ifdef COPMILING_ON_ROBOT
//	argv[4] = "/root/RAN/StereoRobot/src/data/intrinsics.yml";
//	argv[5] = "-e";
//	argv[6] = "/root/RAN/StereoRobot/src/data/extrinsics.yml";
//#else
//	argv[4] = "C:/Users/Ran_the_User/Documents/GitHub/StereoRobot/Src/src/data/intrinsics.yml";
//	argv[5] = "-e";
//	argv[6] = "C:/Users/Ran_the_User/Documents/GitHub/StereoRobot/Src/src/data/extrinsics.yml";
//#endif
//	////outputs:
//	//argv[7] = "-o";	argv[8]  = "../data/disp_out.jpg";
//	//argv[9] = "-p";	argv[10] = "../data/points_out.yml";
//	//outputs:
//	argv[7] = "--no-display";	
//	//argv[8]  = "";	argv[9] = "";	argv[10] = "";
//	localDisp.do_stereo_match_init (argc,argv);
//}

//void do_stereo_disp(Mat imgR, Mat imgL, Mat& outM){
//
//	//inputs:
//	Mat img1 = imgR.clone();
//	Mat img2 = imgL.clone();
//
//	localDisp.do_stereo_match( img1, img2, outM); // openCV demo as base code
//}

