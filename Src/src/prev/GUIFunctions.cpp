/*
GUI_functions.h file
*/
//////////
#ifndef GUI_FUNC_H
#define GUI_FUNC_H

#include "opencv2/opencv.hpp"
#include "myGUI_handler.h" 

using namespace cv;
using namespace std;

void show_vidSource_options_gui();
//#include "utilFunctions.h"
//
////general app options
//enum USER_CHOISE  {
//					USR_PLAY_VID  = 0,  
//					USR_STOP_PLAY = 1,  // PAUSE FOR OPTION TO SIGN TARGET AND SO. FOR VID FLIE OR LIVE CAPTURING(SHOW ANOTHER WINDOW OF CONTINUING FEED)
//					USR_ASIGN_TRACKING_TARGET_MANUALLY = 2 //WHEN VIDEO PAUSED, OR BY 1 PRESS ON MIDDLE BODY WHEN VIDEO PLAYS
//};
//
//// GLOBALS for the GUI stuff
//static const int thumb_num = 10;
//static const int buttons_num = 15;
//String	plotWindowsNames[thumb_num];
//Mat		plotImages[thumb_num]; 
//Point	LastMousePressPos;
//int		userMouseBtnSelection = -1; // 1..buttons_num
//Rect	boundRect[buttons_num];

/* functions headers . partial.. */
//void draw_output_frames(String* WinNames, Mat* images);
//void check_user_input();

/* functions */
//
//// checks where the user pressed on gui window, and returns the
//// number of relevant button indexed it pressed on it area.
//static void onMousePress( int event, int x, int y, int, void* )
//{
// 	switch( event )
//	{
//	case EVENT_LBUTTONDOWN:
//		LastMousePressPos		= Point(x,y);
//		userMouseBtnSelection	= -1;
//
//		for (int i=0 ; i<4; i++){   //buttons_num//TODO: seperate for each gui window.
//			if ( LastMousePressPos.inside (boundRect[i]) )
//			{
//				printf("indeed %d index \n",i); //TODO: update in 'status'  line/section
//				userMouseBtnSelection	= i+1;//
//			}
//		}		
//		break;
//	case EVENT_LBUTTONUP: 
//		break;
//	}
//}
//// for the video source selection window 
//static void onMousePress2( int event, int x, int y, int, void* )
//{
//	if (event == EVENT_LBUTTONDOWN){
//		LastMousePressPos		= Point(x,y);
//		userMouseBtnSelection	= -1;
//
//		for (int i=4 ; i<buttons_num; i++){
//			if ( LastMousePressPos.inside (boundRect[i]) )
//			{
//				printf("indeed %d index \n",i); //TODO: update in 'status'  line/section
//				userMouseBtnSelection	= 30+  i-2;//
//			}
//		}		
//	}
//}

// return user input , or 0(zero) if pressed ESC, -1 as default for no special input.
// p - start/stop play video file. in window 1
// s - set identification to/by current frame
// a - operate/turn-off the camshift algorithm on given frame
// r - re-init video reading.
//
// mouse pressing result treatment. result of user pressing on button 1, or 2 (at the moment only 2 buttons).
//bool myGUI_handler::check_user_input(const int* waiting_delay, char* c)
//{
//	/*keyboard*/
//	*c = (char)waitKey(*waiting_delay);
//	if( *c == 27 )
//		return false;
//	switch(*c)
//	{
//	case 'p':
//		op_flags.play_on					= ! op_flags.play_on ;
//		break;
//	case 's':
//		op_flags.reset_identification		= true;
//		break;
//	case 'a':
//		op_flags.make_camshift				= !op_flags.make_camshift ;
//		break;
//	case 'r':
//		op_flags.reset_vid_file_location	= true;
//		break;
//	}
//
//	/*mouse*/
//	switch (userMouseBtnSelection)
//	{
//	case 1:
//		op_flags.make_stereo_calibration = true;
//		userMouseBtnSelection=-1;
//		break;
//	case 2: // start utils.
//		////////////////////////////////////////////
//		op_flags.calc_background_subs = true;
//		userMouseBtnSelection=-1;
//		////////////////////////////////////////////
//		break;
//	case 3:// choose vid source
//		op_flags.show_vid_source_selection = true;
//	///	show_vidSource_options_gui();
//		userMouseBtnSelection=-1;
//		break;
//	case 4:// show video panels
//		   //// start the displays and wait for tracking
//		op_flags.show_stereo = true;
//		userMouseBtnSelection=-1;
//		break;
//	case 31:
//		thisStereo.input_source = LIVE_CAM;
//		cout << "change video source to LIVE " << thisStereo.input_source << '\n';
//		userMouseBtnSelection=-1;
//		break;
//	case 32:
//		thisStereo.input_source = RECORDED_VIDEO_FILE;
//		cout << "change video source to FILE " << thisStereo.input_source << '\n';
//		userMouseBtnSelection=-1;
//		break;
//	case 33:
//		thisStereo.input_source = IMAGES_LIST;
//		cout << "change video source to IMAGES_LIST " << thisStereo.input_source << '\n';
//		userMouseBtnSelection=-1;
//		break;
//	default: //for calibration or else
//		break;
//	}
//
//
//	return true;
//}


#endif   // GUI_FUNC_H
