#ifndef WORKING_CONSTS_H
#define WORKING_CONSTS_H
/* 
file that contains all working parameters and constants.
*/
using namespace cv;

enum WORKING_MODES	{ CALIBRATION=0, REGULAR  , REG_and_REC , PLAYBACK }; //  REGULAR(=no recording) 
enum FRAME_SIZES	{ SMALL=0, MEDIUM , LARGE};

const int numOfActiveCams		= 1;// use the first # of the next indeces list :
const int camIndexes[3]			= { 0,1 , 2 };       // default : { 0, 1 , 2 };

//const int numOfActiveCams		= 2;// 2;// use the first # of the next indeces list :
//const int camIndexes[3]			= { 1 , 2,0 };       // default : { 0, 1 , 2 };

const int working_FRAME_WIDTH	= 320;// 640;// 160;
const int working_FRAME_HIGHT	= 240;// 480;// 120;

const int REC_FPS				= 15;// 30;	// 10 - will not record
const int REC_CODEC				= CV_FOURCC('D', 'I', 'V', 'X') ; //CV_FOURCC('P', 'I', 'M', '1') ;
const int loop_DELAY = 75;// for 15fps ;// 34; // for camera highest frame rate	//  (int)1000 / REC_FPS; //33;// 10; // [ mS ]  -> 1000/# [Hz]

const int stitching_frame_rate			= 1 ; // 10;	// every # number of frames  - do stitch with previous one.
const int stereo_stitching_frame_rate	= 50 ; // 10;	// every # number of frames  - do stitch between the two stereo ones.

const int CHAR_LEN				= 100;

const char rec_file_name[3][CHAR_LEN]	= { "output_1.avi" , "output_2.avi", "output_3.avi" };
//const char rec_file_name[3][CHAR_LEN] = { "output_1.mp4" , "output_2.avi", "output_3.avi" };
const char window_name[3][CHAR_LEN]		= { "out_1" , "out_2", "out_3" };

const FRAME_SIZES	frame_size   = SMALL;

/* for util functions : */
const String STEREO_CALIBRATION_IMAGES_LIST = "fileList.txt";	// will include files names in left-right pairs order
const String STEREO_CALIBRATION_VIDEO_PAIR  = "output_#.avi";  // '#' will be 1,2 , respectivley for cams 0,1

const int frame_boundary		= 10;	  // for the tracking functions. to not consider the image boundaries.
const int frame_boundary_W_init	= 40;	  // for the movement detection functions. to not consider the image boundaries.

	  /* constants */


enum USER_STATUS_SELECTION {
	JUST_INITIALIZED	=	0 ,		 // mode 0 is the initialization. as NULL
	CAPTURE_CALIBRATION_IMAGES	=	1,			// right - index 0, window title as 1. 	// left - index 1, window title as 2. 
	CALIBRATION_STEREO,
	STREAM_LIVE_FROM_STEREO,
	STREAM_WITH_DISPARITY_AND_DEPTH
};
// 
//enum VIDEO_SOURCE {					// each 2 images will be populated from 
//	STREAM_STEREO_CAMS = 1,		// real-time capture 
//	RECORDED_VIDEOS_COUPLE,		// ready-made couple of video files
//	IMAGES_LIST							// pairs of pre-captured Left-Right images
//};

enum VIDEO_SOURCE {					// each 1 or 2 image frames will be populated from 
	LIVE_CAM			= 0	,		// real-time capture 
	RECORDED_VIDEO_FILE		,		// ready-made (couple) of video files
	IMAGES_LIST						// pairs of pre-captured (Left-Right) images
};

enum CAM_INDECES {RIGHT_CAM=0, LEFT_CAM=1};

struct CalibStruct {
	Size imSize;
	Mat
		CM1, CM2,	// cameraMatrix1,2
		D1, D2,		// distCoeffs1,2   //need to add calibrated imageSize
		R,
		T,
		E,
		F,
		R1, R2,
		P1, P2,
		Q;
};

struct StereoCams
{
	VIDEO_SOURCE	input_source	;
	VideoCapture	cams			[numOfActiveCams];			// set the cameras buffers. 0 for Left, 1 for Right
	CalibStruct		calib_mtx		[numOfActiveCams];			// the calculated/pre-calculated cammeras parameters
	VideoWriter     rec_videos		[numOfActiveCams];		// recordings buffers for stream_input recording

	Mat				raw_frame		[numOfActiveCams];		// intermidiate buffer for capturing
	Mat				modeified_frame	[numOfActiveCams];		// modified buffer for optional processing for each step

	int				vid_len			[numOfActiveCams];		// relevant for pre-recorded input video
	Size			vid_res			[numOfActiveCams];		// relevant for pre-recorded input video

};

StereoCams thisStereo;				// global 

							
// system status to include BackgroundSubstraction & Tracker.
enum SYSTEM_STATUS{
	INITIALIZING	=	0 ,		// Should show GRAY cross	 
	STANDBY			=	1 ,		// Should show ORANGE cross
	FOUND_SOME_MOVEMENT	,
	FOUND_GOOD_TARGET	,
	TRACKING_GOOD_QUALITY_TARGET,		// Should show GREEN cross
	TRACKING_LOW_QUALITY_TARGET,
	TARGET_IS_LOST			// Should show RED cross	,	after 3 sec will turn to ORANGE (while stopping the robot)
};

SYSTEM_STATUS system_state = INITIALIZING ;

#endif //WORKING_CONSTS_H