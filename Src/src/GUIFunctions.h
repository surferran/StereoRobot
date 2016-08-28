/*
GUI_functions.h file
*/
//////////
#ifndef GUI_FUNC_H
#define GUI_FUNC_H

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void show_vidSource_options_gui()
;
//#include "utilFunctions.h"

//general app options
enum USER_CHOISE  {
					USR_PLAY_VID  = 0,  
					USR_STOP_PLAY = 1,  // PAUSE FOR OPTION TO SIGN TARGET AND SO. FOR VID FLIE OR LIVE CAPTURING(SHOW ANOTHER WINDOW OF CONTINUING FEED)
					USR_ASIGN_TRACKING_TARGET_MANUALLY = 2 //WHEN VIDEO PAUSED, OR BY 1 PRESS ON MIDDLE BODY WHEN VIDEO PLAYS
};

struct Operation_flags{
	bool make_stereo_calibration=	false;
	bool calc_background_subs	=	false;
	bool show_vid_source_selection =false;

	bool show_stereo			=	false;
	bool play_on				=	false;
	bool save_disparity_img_to_file = false ; 

	bool make_camshift			=	false;

	bool reset_vid_file_location =	false;
	bool reset_identification	=	false;

	bool show_img_hist			=	false;
	bool proces_img_frame		=	false;
	bool draw_middle_x			=	true;
}op_flags;

// GLOBALS for the GUI stuff
static const int thumb_num = 10;
static const int buttons_num = 15;
String	plotWindowsNames[thumb_num];
Mat		plotImages[thumb_num]; 
Point	LastMousePressPos;
int		userMouseBtnSelection = -1; // 1..buttons_num
Rect	boundRect[buttons_num];

/* functions headers . partial.. */
void draw_output_frames(String* WinNames, Mat* images);
//void check_user_input();

/* functions */
void draw_output_frames(String* WinNames, Mat* images)
{
	for (int i=0; i<thumb_num ; i++)
	{
		imshow( WinNames[i]	, images[i] );
	}
}

// checks where the user pressed on gui window, and returns the
// number of relevant button indexed it pressed on it area.
static void onMousePress( int event, int x, int y, int, void* )
{
 	switch( event )
	{
	case EVENT_LBUTTONDOWN:
		LastMousePressPos		= Point(x,y);
		userMouseBtnSelection	= -1;

		for (int i=0 ; i<4; i++){   //buttons_num//TODO: seperate for each gui window.
			if ( LastMousePressPos.inside (boundRect[i]) )
			{
				printf("indeed %d index \n",i); //TODO: update in 'status'  line/section
				userMouseBtnSelection	= i+1;//
			}
		}		
		break;
	case EVENT_LBUTTONUP: 
		break;
	}
}
// for the video source selection window 
static void onMousePress2( int event, int x, int y, int, void* )
{
	if (event == EVENT_LBUTTONDOWN){
		LastMousePressPos		= Point(x,y);
		userMouseBtnSelection	= -1;

		for (int i=4 ; i<buttons_num; i++){
			if ( LastMousePressPos.inside (boundRect[i]) )
			{
				printf("indeed %d index \n",i); //TODO: update in 'status'  line/section
				userMouseBtnSelection	= 30+  i-2;//
			}
		}		
	}
}

// return user input , or 0(zero) if pressed ESC, -1 as default for no special input.
// p - start/stop play video file. in window 1
// s - set identification to/by current frame
// a - operate/turn-off the camshift algorithm on given frame
// r - re-init video reading.
//
// mouse pressing result treatment. result of user pressing on button 1, or 2 (at the moment only 2 buttons).
bool check_user_input(const int* waiting_delay, char* c)
{
	/*keyboard*/
	*c = (char)waitKey(*waiting_delay);
	if( *c == 27 )
		return false;
	switch(*c)
	{
	case 'p':
		op_flags.play_on					= ! op_flags.play_on ;
		break;
	case 's':
		op_flags.reset_identification		= true;
		break;
	case 'a':
		op_flags.make_camshift				= !op_flags.make_camshift ;
		break;
	case 'r':
		op_flags.reset_vid_file_location	= true;
		break;
	}

	/*mouse*/
	switch (userMouseBtnSelection)
	{
	case 1:
		op_flags.make_stereo_calibration = true;
		userMouseBtnSelection=-1;
		break;
	case 2: // start utils.
		////////////////////////////////////////////
		op_flags.calc_background_subs = true;
		userMouseBtnSelection=-1;
		////////////////////////////////////////////
		break;
	case 3:// choose vid source
		op_flags.show_vid_source_selection = true;
	///	show_vidSource_options_gui();
		userMouseBtnSelection=-1;
		break;
	case 4:// show video panels
		   //// start the displays and wait for tracking
		op_flags.show_stereo = true;
		userMouseBtnSelection=-1;
		break;
	case 31:
		thisStereo.input_source = LIVE_CAM;
		cout << "change video source to LIVE " << thisStereo.input_source << '\n';
		userMouseBtnSelection=-1;
		break;
	case 32:
		thisStereo.input_source = RECORDED_VIDEO_FILE;
		cout << "change video source to FILE " << thisStereo.input_source << '\n';
		userMouseBtnSelection=-1;
		break;
	case 33:
		thisStereo.input_source = IMAGES_LIST;
		cout << "change video source to IMAGES_LIST " << thisStereo.input_source << '\n';
		userMouseBtnSelection=-1;
		break;
	default: //for calibration or else
		break;
	}


	return true;
}


/////////////////////////////

//int to string helper function
string _intToString(int number){
	 
	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}
//double to string helper function
string _doubleToString(double number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//system status enum to string helper function
string _sysStatToString(SYSTEM_STATUS val){

	std::stringstream ss;
	switch (val)
	{
	case INITIALIZING:
		ss << "sys : INITIALIZING";
		break;
	case STANDBY:
		ss << "sys : STANDBY";
		break;
	case FOUND_SOME_MOVEMENT:
		ss << "sys : MOVEMENT";
		break;
	case TRACKING_GOOD_QUALITY_TARGET:
		ss << "sys : TRACKING_GOOD";
		break;
	case TRACKING_LOW_QUALITY_TARGET:
		ss << "sys : TRACKING_LOW";
		break;
	case TARGET_IS_LOST:
		ss << "sys : LOST";
		break;
	default:
		break;
	}
	return ss.str();
}

/////////////////////////////

/*
add sign of cross on the x,y point, on the delivered image.
usually to show Target location, or image center.
*/
void add_Cross_to_Image(int x, int y, bool addLabel, SYSTEM_STATUS sys_stat, Mat &cameraFeed)   
{
	Scalar color = Scalar(0,255,0) ;
	if (sys_stat==INITIALIZING)
		color = Scalar(100, 55, 10) ;
	else
		if (sys_stat==STANDBY)
			color = Scalar(100, 155, 110) ;
	else
		if (sys_stat==FOUND_SOME_MOVEMENT)
			color = Scalar(100, 255, 10) ;
	//draw some crosshairs around the object
	
	circle	(cameraFeed,Point(x,y), 10			,color,2); //r20
	line	(cameraFeed,Point(x,y+25),Point(x,y-25),color,2);
	//line	(cameraFeed,Point(x,y),Point(x,y+25),color,2);
	line	(cameraFeed,Point(x+25,y),Point(x-25,y),color,2);
	//line	(cameraFeed,Point(x,y),Point(x+25,y),color,2);

	// line to image center
	line	(cameraFeed,Point(x,y),Point(cameraFeed.size().width/2 , cameraFeed.size().height/2),color/2,2);

	// TODO: consider print it in the image corner. in status bar for example.
	//write the position of the object to the screen
	///putText(cameraFeed,"Tracking object at (" + _intToString(x)+","+_intToString(y)+")",Point(x,y),1,0.51,Scalar(255,0,0),1);
	if (addLabel)
		putText(cameraFeed,"Tracking object at (" + _intToString(x)+","+_intToString(y)+")",
			Point(x/20+1,y/20+20),1,0.51,Scalar(255,0,0),1);

}

#endif   // GUI_FUNC_H
