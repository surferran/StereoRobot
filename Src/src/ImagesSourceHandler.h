
#ifndef IMAGESSOURCEHANDLER_H_
#define IMAGESSOURCEHANDLER_H_

#include "opencv2/opencv.hpp"

using namespace cv;

#include <thread>
#include <mutex>
#include <atomic>

#ifdef COMPILING_ON_ROBOT
#define ACTIVE_CAMS_NUM			2
#define LEFT_CAMERA_INDEX		1		// depends on platform. 0 index is the default camera.
#define RIGHT_CAMERA_INDEX		0
#else

#ifdef RUN_ON_LAPTOP__MONO 

#define ACTIVE_CAMS_NUM			1
#define LEFT_CAMERA_INDEX		0		// depends on platform. 0 index is the default camera.
#define RIGHT_CAMERA_INDEX		1

#else 

#define ACTIVE_CAMS_NUM			2
#define LEFT_CAMERA_INDEX		0		// depends on platform. 0 index is the default camera.
#define RIGHT_CAMERA_INDEX		2

#endif

#endif

class ImagesSourceHandler 
{
private: 
	/* the image source and its properties */
	VideoCapture	vidR,		// handles the Hardware right camera
					vidL;		// handles the Hardware left camera

	const int		w=320,		// desired resolution for the images
					h=240;

	bool			recordFlag;

	/* the output images and properties */
	//const int		N = 10;		// number of frames to keep history for
	//int			frame_cyclic_counter = 0;
    //Mat left[N],
	//	right[N];
	
	Mat left, right;
	double captureTimeTag;

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
	//TODO: //void setRecordOption()
};

#endif /* IMAGESSOURCEHANDLER_H_ */
