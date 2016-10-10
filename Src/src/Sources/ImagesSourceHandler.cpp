
#include "ImagesSourceHandler.h"
#ifdef COMPILING_ON_ROBOT
#include <unistd.h>
#endif
 

/* initializing , and executing the thread */
ImagesSourceHandler::ImagesSourceHandler() :
	videoGrab_thrd(&ImagesSourceHandler::CaptureFromCam,this) 
{}

void ImagesSourceHandler::CaptureFromCam() {
	InitVideoCap();
	userDisableFramesCapture = false;
	exit = false;
	while (!exit) {
		mut.lock();

		if (!userDisableFramesCapture)
		{
			vidL >> left;
			if (ACTIVE_CAMS_NUM==2)
				vidR >> right;
			else
				right=left; 
			//TODO: //captureTimeTag = .. 

			inputFrameCycleCounter++; //TODO: condition with non empty frames?!

			/*if (bUserRecordRequest)	// moved to recieving function
			{
				outFileL.write(left);
				outFileR.write(right);
			}
			*/
		}
		mut.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(capture_loop_dealy));
	}
}

void ImagesSourceHandler::InitVideoCap() 
{ 
	StereoRobotApp obj;

	w	=	obj.working_FRAME_WIDTH ;	// desired resolution for the images
	h	=	obj.working_FRAME_HIGHT ;
	FPS	=	obj.working_FRAMES_FPS;		// 30, or 15  -> capture_loop_dealy=33, or 67 [mS]

#ifdef COMPILING_ON_ROBOT
	bRepeat_scenario_from_files	=	false;//true;//true;//false; 
	bUserRecordRequest			=	true;//false;true;//
#else
	bRepeat_scenario_from_files	=	true;//false;//true;//true;//false; 
	bUserRecordRequest			=	false;//true;//false;true;//
#endif

	strcpy(inout_file_nameR , "../vidR.avi");
	strcpy(inout_file_nameL , "../vidL.avi");

	/* initiate */
	captureTimeTag = 0;
	if (bRepeat_scenario_from_files)
	{
		vidL.open(inout_file_nameL);
		vidR.open(inout_file_nameR);
	}
	else
	{
		vidL.open(LEFT_CAMERA_INDEX);
		if (ACTIVE_CAMS_NUM==2)
			vidR.open(RIGHT_CAMERA_INDEX);	
	}
	
	/* checks */
	if(!vidL.isOpened())	throw cv::Exception(1, "Cannot open right camera/file \n" , "", "", 30);///std::exception(errMsg2.str().c_str());
	if (ACTIVE_CAMS_NUM==2)
		if(!vidR.isOpened())	throw cv::Exception(); //std::exception(errMsg1.str().c_str()); //throw cv::Exception();
	
	/* settings */
	if (!bRepeat_scenario_from_files) {
		vidL.set(CAP_PROP_FRAME_WIDTH, w);	vidL.set(CAP_PROP_FRAME_HEIGHT, h); vidL.set(CAP_PROP_FPS, FPS); 
		if (ACTIVE_CAMS_NUM==2){
			vidR.set(CAP_PROP_FRAME_WIDTH, w);	vidR.set(CAP_PROP_FRAME_HEIGHT, h);  vidR.set(CAP_PROP_FPS, FPS); }
	}
	else; //TODO: update FPS from recorded files. (although it should be the same as the last setting that recorded.

	/* option for videos recording to files */
	if (bUserRecordRequest)
	{
		if (!bRepeat_scenario_from_files)		// cannot record to files while also images are input from same files
		{
			//init files to record to
			outFileR.open(inout_file_nameR, codec, FPS/2, Size(w,h), true);
			outFileL.open(inout_file_nameL, codec, FPS/2, Size(w,h), true);

			if (!outFileR.isOpened() || !outFileL.isOpened()) {
				throw cv::Exception(); 
				//cerr << "Could not open the output video file for write\n";
				//return -1;
			}				
		}
		else
			bUserRecordRequest = false; //+error..
	}

	/* about 40degree for w/2 pixels .  get angle coefficient by that */
	camFOV		=	40.0 * 1. / 57.3 ; // deg to rad conversion
	camFOVpix	=	camFOV / (w/2);
}


bool ImagesSourceHandler::GetFrames(Mat &rightFrame, Mat &leftFrame)	// can b replaced with shared_memory?!
{
	mut.lock();

	leftFrame  = left;
	rightFrame = right;

	if ( rightFrame.empty()  ||  leftFrame.empty() )
	{
		//TODO: check TimeOut - if crooses timeout - then stop and ERROR
		mut.unlock();
		return false;
	}
	/* else : */

	recievedFramesCounter++;

	if (bUserRecordRequest)		
		{
			outFileL.write(leftFrame);
			outFileR.write(rightFrame);
		}

	// TODO:
	// make output fot the 'captureTimeTag'
	// check if needed also gray level images

	mut.unlock();

	return true;
}

bool ImagesSourceHandler::setStartingFrame(int initialFrameIndex)
{
	if (bRepeat_scenario_from_files) 
	{		
		vidL.set(CV_CAP_PROP_POS_FRAMES, initialFrameIndex);	
		vidR.set(CV_CAP_PROP_POS_FRAMES, initialFrameIndex); 
		return true;
	}
	else
		return false;
}

ImagesSourceHandler :: ~ImagesSourceHandler()
{
	exit = true;
	videoGrab_thrd.join();
	// also - the videofile will be closed and released automatically in VideoWriter destructor
}
