#ifndef MYGUI_HANDLER_H
#define MYGUI_HANDLER_H

#include "opencv2/opencv.hpp"

//#include "working_consts.h"
#include "StereoRobotApp.hpp"

using namespace cv;
using namespace std;

void cvShowManyImages(char* title, int nArgs, ...) ;   //#include "showManyImages.cpp"

class myGUI_handler
{
public:
	myGUI_handler();
	~myGUI_handler(){};

	void draw_output_frames(String* WinNames, Mat* images);

	void show_graphics_with_image(Mat & mask, Point MassCenter, double rCircle, Rect boundRect, 
									double theta, double boundAreaRatio, int mask_status, int frame_counter);

	void add_counterFrame(Mat &inImage, long * frameNum);

	void add_Cross_to_Image(int x, int y, bool addLabel, StereoRobotApp::SYSTEM_STATUS sys_stat, Mat &cameraFeed);

	void add_distance_to_disparityIM(double dist, Mat &ImFeed);

	void dispFlowChanges(Mat & prevGrayROI, vector<Point2f> trackedFeatures, Mat& grayROI, vector<Point2f> corners,
							vector<uchar> status);

	void display_rectified_pair(Size imageSize , Mat Rimg, Mat Limg, Rect validROI1, Rect validROI2  , long FrameCtr );
	///bool check_user_input(const int* waiting_delay, char* c);
	void onMouseWSHED( int event, int x, int y, int flags, void* );

	void show_raw_captures(Mat L_in, Mat R_in, long frameCounter);
	void show_disparity_map(Mat sum_of_disp, int avg_depth);

	static const int	thumb_num = 10;	
	String				plotWindowsNames[thumb_num];
	Mat					plotImages[thumb_num]; 

	static const int	buttons_num = 15;
	Point				LastMousePressPos;
	int					userMouseBtnSelection = -1; // 1..buttons_num
	Rect				boundRect[buttons_num];

	/* for WaterShed use */
	Mat		markerMask_WSHED, img_WSHED;
	Point	prevPt_WSHED;//(-1, -1);

	enum {
		WIN1_NDX = 0,
		WIN2_NDX,
		WIN3_NDX,
		WIN4_NDX,
		NUM_OF_GUI_WINDOWS
	};

private:
		//TODO: change those to overloading or use template
	string _longToString(long number);
	string _intToString(int number);
	string _doubleToString(double number);
	string _sysStatToString(StereoRobotApp::SYSTEM_STATUS val);
};

#endif   //  MYGUI_HANDLER_H