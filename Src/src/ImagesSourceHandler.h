
#ifndef IMAGESSOURCEHANDLER_H_
#define IMAGESSOURCEHANDLER_H_

#include "opencv2/opencv.hpp" 

#include "working_consts.h"		// my added definitions, constants

using namespace cv;

#include <thread>
#include <mutex>
#include <atomic>

///
#define RUN_ON_LAPTOP__MONO 

#ifdef COMPILING_ON_ROBOT
#define ACTIVE_CAMS_NUM			2
#define LEFT_CAMERA_INDEX		1		// depends on platform. 0 index is the default camera.
#define RIGHT_CAMERA_INDEX		0

#else 

#ifdef RUN_ON_LAPTOP__MONO

#define RUN_ON_LAPTOP__MONO		true 
#define ACTIVE_CAMS_NUM			1
#define LEFT_CAMERA_INDEX		0		// depends on platform. 0 index is the default camera.
#define RIGHT_CAMERA_INDEX		1

#else 

#define RUN_ON_LAPTOP__MONO		false     
#define ACTIVE_CAMS_NUM			2
#define LEFT_CAMERA_INDEX		2		// depends on platform. 0 index is the default camera.
#define RIGHT_CAMERA_INDEX		1

#endif

#endif

class ImagesSourceHandler 
{
private: 
	/* the image source and its properties */
	VideoCapture	vidR,		// handles the Hardware right camera
					vidL;		// handles the Hardware left camera

	const int		w	=	working_FRAME_WIDTH ,		// desired resolution for the images
					h	=	working_FRAME_HIGHT ,
					FPS	=	30;		// 30, or 15  -> capture_loop_dealy=33, or 67 [mS]

	/* vars for input and output from/to files */
	bool			bRepeat_scenario_from_files	=	false;//false;	// true for input images from previously recorded files.
	bool			bUserRecordRequest			=	false;//false;	// true for record captured images to files.
	char			inout_file_nameR[150]	= "../vidR.avi";	//string	//relative to 'StereoRobot\Src\StereoRobot' (in VS2015)
	char			inout_file_nameL[150]	= "../vidL.avi";	//			//
	VideoWriter		outFileR,
					outFileL;
	int				codec					= CV_FOURCC('M', 'J', 'P', 'G'); 

	/* the output images and properties */
	//const int		N = 10;		// number of frames to keep history for
	//int			frame_cyclic_counter = 0;
    //Mat left[N],
	//	right[N];
	
	Mat		left, right;
	double	captureTimeTag;

	/* local variables to use */
	const int		capture_loop_dealy = 33 ; //[mS] , delay between capture cycles
	std::mutex		mut;
	std::thread		videoGrab_thrd;
	//atomic<bool>
		bool exit; 

public:
	ImagesSourceHandler();			// also opens a new thread for this 
	~ImagesSourceHandler();			// also closes the thread for this
	void CaptureFromCam();			// the overall-wrapping function for the thread
	void InitVideoCap();
    void GetFrames(Mat& rightFrame,Mat& leftFrame);
	Size GetRes() { return Size(w,h); };
	//TODO: //void setRecordOption()
};

#endif /* IMAGESSOURCEHANDLER_H_ */
