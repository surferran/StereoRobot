/*
 *  Depth_and_Disparity.cpp 
 */

#ifdef COMPILING_ON_ROBOT
#include "Depth_and_Disparity.hpp"
#else
#include "..\Headers\Depth_and_Disparity.hpp"
#endif

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

Depth_and_Disparity::Depth_and_Disparity():
	/* initializing , and executing the thread */
	stereoIm_thrd(&Depth_and_Disparity::thread_loop_function,this)  
{}

Depth_and_Disparity :: ~Depth_and_Disparity()
{
	exit = true;
	stereoIm_thrd.join();
}

/////////////////////////////////////////////////////////////////////////

void Depth_and_Disparity::thread_loop_function() {

	int		argc;
	char*	argv[11];  
	StereoRobotApp obj;

	Size imgSize= Size(obj.working_FRAME_WIDTH,	obj.working_FRAME_HIGHT);	// desired resolution for the images, same as in ImageSourceHandler 


	argc = 8;//11;

	argv[3] = "-i";
#ifdef COMPILING_ON_ROBOT
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

int Depth_and_Disparity::stereo_match_and_disparity_init(int argc, char** argv,  Size img_size)
{  
	if(argc < 3)  
    { 
        return 0;
    } 

	//alg						= STEREO_SGBM;	
	alg						=	 STEREO_BM;
	desired_param_set		=	3;			// called at the end of this function.

	minDisparityToCut		=	35;		// for the threshold cut	//default here. runover later in ParamFunction.

	SADWindowSize			= 0;
	numberOfDisparities		= 0;
	no_display				= false;
	scale					= 1.f;

	target_image_size		=	img_size;

	last_disparity_depth= 0;

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
		Size img_ORG_size = Size(320,240); // the images size when calibrated the cameras
		stereoRectify( M1, D1, M2, D2, img_ORG_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

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
void Depth_and_Disparity::set_BM_params_options_1()
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
void Depth_and_Disparity::set_BM_params_options_2()
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
void Depth_and_Disparity::set_BM_params_options_3()
{   
	///numberOfDisparities	=	  ((target_image_size.width/8) + 15) & -16;   //  /8..-> 48,   /8/4.. -> 16 disperities
	//numberOfDisparities	=	  ((target_image_size.width/8)*3 + 15) & -16;   // -> 128 disperities
	numberOfDisparities	=	  ((target_image_size.width/8)*3 - 15) & -16;   // -> 96 disperities, relevant for 320x240
	///numberOfDisparities = 48;// good for small resolution of 80x60
	bm->setPreFilterSize	( 35 ); //41
	bm->setPreFilterCap		( 31 );
	bm->setBlockSize		( 35 );//SADWindowSize //41
	bm->setMinDisparity		(-numberOfDisparities/2 );
	bm->setNumDisparities	( numberOfDisparities );
	bm->setTextureThreshold	( 10 );
	bm->setUniquenessRatio	( 15 );

	minDisparityToCut		=	17;		// for the threshold cut	//20 for 2.5m
										//35 is for about 140cm // 18 about 4 meters, 12 about 5m

	/*bm->setROI1(roi1);
	bm->setROI2(roi2);
	bm->setBlockSize( > 0 ? SADWindowSize : 9);	
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);*/

}

//// set parameters according to some recomandations 
void Depth_and_Disparity::set_SGBM_params_options_1()
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
void Depth_and_Disparity::set_SGBM_params_options_2()
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

int Depth_and_Disparity::do_stereo_match(Mat imgR, Mat imgL , Mat& disp8 )
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
		///http://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
///#ifndef COMPILING_ON_ROBOT
		if (1==2)		
		{
			Mat xyz_again;
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

void Depth_and_Disparity::set_disparity_input(Mat inR, Mat inL, long relevantCycleCounter)
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

bool Depth_and_Disparity::get_rectified_and_disparity(Mat& disp_output, rectification_outputs& rectified_vars)
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


void Depth_and_Disparity::convert_disperity_value_to_depth(double in_disp, double & out_depth)
{

	// manual settings
	double camera_Base     = 0.06 ; //[m]
	double Focal_lenght    = 375 ;	//[pix]
	double constant_offset = 0;		//[m] 
	double scale_factor    = 255;//67.852222393615605;/// as 1/W 	// homogenic depth (Z/w) to real depth (Z) by *1/w

	//perspectiveTransform?

	/* bf/d */
	if (in_disp > 0)
		out_depth  = constant_offset + (1./in_disp) * camera_Base * Focal_lenght * scale_factor; //[cm] because of the calibration factor
	else
		out_depth = 9990;

	// add by 	perspectiveTransform ?

}

void Depth_and_Disparity::convert_disperity_Mat_to_depth(Mat in_disp, Mat & out_depth)
{
	reprojectImageTo3D(in_disp, out_depth, Q, true); 
	//Vec3f point_middle = xyz_again.at<Vec3f>(xyz_again.rows/2, xyz_again.cols/2);
	//printf("\n\n middle point relative coor. are: %f %f %f \n\n", point_middle.val[0],point_middle.val[1],point_middle.val[2]);
}

///////////////////////////
/* desiredPhase :
	1 - return raw disparity
	2 - return only filtered disparity for last captured disparity
	3 - make the whole process - calculate disparity and return the filtered one
*/
//bool Depth_and_Disparity::calc_disperity(int desiredPhase, Mat in_left_clr, Mat in_right_clr, 
bool Depth_and_Disparity::calc_disperity(int desiredPhase, Mat left_im_gray, Mat right_im_gray, 
											Mat BgMask , Target *previousTarget,
											Mat *disperity_out, double *min_depth_of_ROI)
{
	double		max_disperity;
	const double allowed_disp_delta_between_cycles = 0.1 ; //10[%]

	if ( ! (desiredPhase==2) )		//calculate the new disparity for new inputs
	{
		// delivers new input , when the process is waiting (not in calculation process)
		set_disparity_input(right_im_gray,left_im_gray, /*myStereoCams.GetFrameCycleCounter()*/ 1 );  

		// waiting trial : 
		while (! get_rectified_and_disparity(last_result_of_disparity, last_result_of_disparity_struct) )  
		{
		}

		if ((desiredPhase==1))
		{
			*disperity_out = last_result_of_disparity.clone() ;
			return true;
		}
	}
	// continue to give the filtered disparity (for the new or the last calculated)

	/* if output is ready from disparity calculation , it returns true */
	//if ( localDisp.get_rectified_and_disparity(disp_temporary, disperity_struct) )  
	{
		/* calculate average depth for the ROI of the target */ 
		Mat tmpma = last_result_of_disparity;
		if ( ! BgMask.empty() )
		{
			filtered_disparity =	Mat();
			last_result_of_disparity.copyTo(filtered_disparity , BgMask);
			threshold (filtered_disparity , filtered_disparity ,	minDisparityToCut ,	255,THRESH_TOZERO);	
		}
		else 
			/* no additional mask */
			if ((*previousTarget).target_object_prop.relevant_disparity > -999)
			{
				threshold (last_result_of_disparity , filtered_disparity ,	
					(*previousTarget).target_object_prop.relevant_disparity * (1-allowed_disp_delta_between_cycles) ,	
					(*previousTarget).target_object_prop.relevant_disparity * (1+allowed_disp_delta_between_cycles),
					THRESH_TOZERO);	//.15??	
			}
			else
			/* loose the far away objects (small disparities) */
			threshold (last_result_of_disparity , filtered_disparity ,	minDisparityToCut ,	255,THRESH_TOZERO);	

		int an=1;	//an=1->kernel of 3
		Mat element = getStructuringElement(MORPH_RECT, Size(an*2+1, an*2+1), Point(an, an) );
		medianBlur	(filtered_disparity,	filtered_disparity,	an*3);
		erode		(filtered_disparity ,	filtered_disparity, element);									
		dilate		(filtered_disparity,	filtered_disparity, element); 

		//max disperity into avg_disp var
	//	minMaxLoc(filtered_disparity, 0, &max_disperity ); 
	//	convert_disperity_value_to_depth(max_disperity , *min_depth_of_ROI);	
	//	last_disparity_min_depth	= *min_depth_of_ROI;


		Scalar     mean;
		Scalar     stddev;
		Rect tmp = boundingRect(filtered_disparity);
		Mat  tmpMask = filtered_disparity;
		///meanStdDev ( filtered_disparity, mean, stddev );
		///meanStdDev ( filtered_disparity(tmp), mean, stddev );
		meanStdDev ( filtered_disparity, mean, stddev , tmpMask);
		//uchar       mean_pxl = mean.val[0];
		int         mean_val = mean.val[0];
		//uchar       stddev_pxl = stddev.val[0];
		int         stddev_val = stddev.val[0];

		int minDispToTake = mean_val - stddev_val * 1 ;
		if (minDispToTake > mean_val * (1-allowed_disp_delta_between_cycles/2.) ) 
			minDispToTake = mean_val * (1-allowed_disp_delta_between_cycles/2.) ; //minimum for case of ~zero std ;	// furthest object
		//int maxDispToTake = max_disperity ;						// closest  object

		convert_disperity_value_to_depth(mean_val , *min_depth_of_ROI);	
		last_disparity_depth	= *min_depth_of_ROI;

		/* filter far or close objects then the target itself (mean) */
		threshold (filtered_disparity , filtered_disparity ,	minDispToTake ,	255/*max_disperity*/,THRESH_TOZERO);
		/* set partial data for current target */
		(*previousTarget).target_object_prop.relevant_disparity = mean_val;
	}
	
	*disperity_out = filtered_disparity.clone() ;
	
	return true;	// TODO: set as SUCCESS system enum
}


void Depth_and_Disparity::get_filtered_disparity(Mat &dispOut, int *avg_Depth)
{
	dispOut		= filtered_disparity.clone();
	*avg_Depth	= (int)last_disparity_depth;
}
