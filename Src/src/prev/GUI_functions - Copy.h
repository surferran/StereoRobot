/*
GUI_functions.h file
*/
//////////

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

#include "utilFunctions.h" 

enum USER_CHOISE  {
					USR_PLAY_VID  = 0,  
					USR_STOP_PLAY = 1,  // PAUSE FOR OPTION TO SIGN TARGET AND SO. FOR VID FLIE OR LIVE CAPTURING(SHOW ANOTHER WINDOW OF CONTINUING FEED)
					USR_ASIGN_TRACKING_TARGET_MANUALLY = 2 //WHEN VIDEO PAUSED, OR BY 1 PRESS ON MIDDLE BODY WHEN VIDEO PLAYS
};

struct Operation_flags{
	bool play_on				= false;
	bool make_camshift			= false;
	bool reset_vid_file_location = false;
	bool reset_identification	= false;
	bool show_img_hist			= false;
	bool proces_img_frame		= false;
}op_flags;

// GLOBALS
static const int thumb_num = 10;
String	plotWindows[thumb_num];
Mat		images[thumb_num]; 
Point	LastMousePressPos;
int		userPressedMouse = -1;
Rect	boundRect[20];

/* functions headers */
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

static void onMousePress( int event, int x, int y, int, void* )
{
 	switch( event )
	{
	case EVENT_LBUTTONDOWN:
		LastMousePressPos	= Point(x,y);
		userPressedMouse	= -1;//true;

		if ( LastMousePressPos.inside (boundRect[0]) )
		{
			printf("indeed 0 ");
			userPressedMouse	= 1;//
		}
		break;
	case EVENT_LBUTTONUP: 
		break;
	}
}

// return user input , or 0(zero) if pressed ESC, -1 as default for no special input.
// p - start/stop play video file. in window 1
// s - set identification to/by current frame 
// a - operate/turn-off the camshift algorithm on given frame
// r - re-init video reading.
bool check_user_input(int* waiting_delay, char* c)   
{
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
	case 'd':
		
		break;

	//default: ;
	}
	return true;
}


void show_buttons_gui()
{
	String WinName = "User selections";
	namedWindow(WinName, CV_WINDOW_AUTOSIZE); //create a window 
	Mat btns_im(120, 340, CV_8UC3, Scalar(10 , 10 , 10 ));  // hight, width,type,.. //create_empty_image() 
	Mat btn_im ( 20,  310, CV_8UC3, Scalar(100, 100, 100));  // hight, width,type,.. //create_empty_image() 

	// TODO: add text to show working directory. 
	// and calibration files directory.
	// and options possibilities (the enum)

	setMouseCallback(WinName , onMousePress, 0 );

	std::string text	= " show stereo stream ";
	setLabel(btn_im, text, cvPoint(5, 13)); //gives x,y as left lower // return is w,h + l,top	//. or set boundaries for buttons table
	
	boundRect[0] = Rect (+ 13, + 5,320,20);//x,y,w,h
	//boundRect[0].
	//btn_arr[0].index = 0;
	//btn_arr[0].text = text;
	//btn_arr[0].Val = 0;
	//btn_arr[0].x_min = 10;
	//btn_arr[0].x_max = 10 + boundary.width;
	//btn_arr[0].y_min = 30;
	//btn_arr[0].y_max = 30 + boundary.height;
	/**/

	int originX = 10,
		originY = 10 ;
	Rect roi( Point( originX, originY ), btn_im.size() );
	btn_im.copyTo( btns_im( roi ) );


	btn_im=Mat  ( 20,  310, CV_8UC3, Scalar(100, 100, 100));//clear
	text				= " capture calibration images from BW stream ";
	setLabel(btn_im, text, cvPoint(5, 13));	//attach callback. or set boundaries for buttons table
	//btn_arr[1].index = 1;
	//btn_arr[1].text = text;
	//btn_arr[1].Val = 0;
	//btn_arr[1].x_min = 10;
	//btn_arr[1].x_max = 10 + boundary.width;
	//btn_arr[1].y_min = 30;
	//btn_arr[1].y_max = 30 + boundary.height;
	/* when hover above - change background. (redisplay label with different bckg color )*/

	originX = 10 ;
	originY = 40 ;
	roi = Rect( Point( originX, originY ), btn_im.size() );
	btn_im.copyTo( btns_im( roi ) );

	imshow(WinName, btns_im);

}

