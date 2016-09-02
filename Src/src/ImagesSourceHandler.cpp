
#include "ImagesSourceHandler.h"

//TODO: #ifdef COMPILING_ON_ROBOT
///#include <unistd.h>


ImagesSourceHandler::ImagesSourceHandler() :
		/* initializing , and executing the thread */
	videoGrab_thrd(&ImagesSourceHandler::CaptureFromCam,this) 
{}

void ImagesSourceHandler::GetFrames(Mat &rightFrame, Mat &leftFrame)	// can b replaced with shared_memory?!
{
    mut.lock();
    leftFrame  = left;
    rightFrame = right;
	// make output fot the 'captureTimeTag'
    mut.unlock();
}

void ImagesSourceHandler::InitVideoCap() 
{ 
	/* initiate */
	captureTimeTag = 0;
	vidL.open(LEFT_CAMERA_INDEX);
	if (ACTIVE_CAMS_NUM==2)
		vidR.open(RIGHT_CAMERA_INDEX);	
	 
	/* checks */
	if(!vidL.isOpened())	throw cv::Exception(1, "Cannot open right camera\n" , "", "", 30);///std::exception(errMsg2.str().c_str());
	if (ACTIVE_CAMS_NUM==2)
		if(!vidR.isOpened())	throw cv::Exception(); //std::exception(errMsg1.str().c_str()); //throw cv::Exception();
	
	/* settings */
	vidL.set(CAP_PROP_FRAME_WIDTH, w);	vidL.set(CAP_PROP_FRAME_HEIGHT, h);
	if (ACTIVE_CAMS_NUM==2){
		vidR.set(CAP_PROP_FRAME_WIDTH, w);	vidR.set(CAP_PROP_FRAME_HEIGHT, h);}
}

void ImagesSourceHandler::CaptureFromCam() {
	InitVideoCap();
	exit = false;
	while (!exit) {
        mut.lock();
		vidL >> left;
		if (ACTIVE_CAMS_NUM==2)
			vidR >> right;
		else
			right=left; 
		//TODO: //captureTimeTag = ..   
        mut.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(capture_loop_dealy));
    }
}

ImagesSourceHandler :: ~ImagesSourceHandler()
{
	exit = true;
	videoGrab_thrd.join();
}
