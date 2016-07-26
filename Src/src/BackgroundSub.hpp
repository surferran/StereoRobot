#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <stdio.h>

//#include "opencv2/opencv.hpp"
#include "GUI_functions.h"		// for using _intToString

using namespace cv;

class BackSubs
{
public:

	BackSubs(){};
	int show_forgnd_and_bgnd_init(VideoCapture vidSource_LeftCam);
	int show_forgnd(Mat frame);
	Mat get_foreground_mat() { return foreground; } ;

	SYSTEM_STATUS BackSubs_State = INITIALIZING ;

private:

	int show_more_details(Mat frame) ;

	VideoCapture			cap;
	String					vidName		= ""; 
	String					StatusText	= "NAN";

	int						fps;
	Mat						frame, foreground, image;	// inner vars for class functions.
	Mat						middle_tmp_frame;			//	
	vector<vector<Point> >	contours;
	vector<vector<Point> >	selected_shapes_contours;
	int						area;
	Rect					rect;
    
	cv::Ptr<BackgroundSubtractorMOG2> mog ;
	Scalar					random_color;
	int						loopWait = 0;
	int						found_contours_num ;

	const int				BackSubs_History		= 120;
	const double			BackSubs_Threshould		= 16.0;	
	const bool				BackSubs_DetectShadows	= false;   // only for better run-time performance

};

	RNG						rng(12345);
// the function draws all the squares in the image
//static void drawShapesContours(Mat& image, const vector<vector<Point> >& ShapesContours)
static void drawShapesContours(Mat& image, const vector<vector<Point> >& ShapesContours,
	const vector<Moments> & curves_moments, vector<Point2f>& mass_centers)
{
	for (size_t i = 0; i < ShapesContours.size(); i++)
	{
		const Point*	p = &ShapesContours[i][0];
		int				n = (int)ShapesContours[i].size();
		polylines(image, &p, &n, 1, true, Scalar(255, 255, 0), 3, LINE_AA);

		if (!mass_centers.empty()) {
			circle(image, mass_centers[i], 4, Scalar(0, 255, 255), -1, 8, 0);
		}
		if (!curves_moments.empty()) {
			//draw text of data about it, near the mass center 
		}
	}

	imshow("Capture ", image);
}

// TODO: return parameters of rCircle, boundRect, theta, frame_counter(of bkgSubs) (as part of class?)
// calculate and print some parameters for the current frame foreground
int doMYbsManipulation( Mat & mask)
{ 
	static int frame_counter=0;
	int mask_status = 0;

	Moments m = moments(mask, false);	// points moment 
	Point p1(m.m10/m.m00, m.m01/m.m00); // mass_centers

	double rCircle = sqrt(m.m00/3.14)/13 ; //10~  // estimated rounding circle for the object area
	circle(mask, p1, rCircle, Scalar(128,220,220), 3); 

	Rect boundRect = boundingRect ( mask );
	rectangle(mask,
		boundRect.tl(), boundRect.br(),
		Scalar(128,220,220) ,	2, 8, 0);

	double theta = 0.5 * atan2(2*m.m11, m.m20-m.m02) * 57.3;
	String StatusText = "theta=" + _doubleToString(theta);
	putText(mask, StatusText, Point(15, 15), FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 255), 1);
	       StatusText = "rCircle=" + _doubleToString(rCircle);
	putText(mask, StatusText, Point(15, 25), FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 255), 1);

	double tmp1 = 100. * boundRect.area() ;
	double tmp2 = (mask.size()).area() ; 
	mask_status = tmp1 / tmp2;
	StatusText  = "status=" + _intToString(mask_status);
	putText(mask, StatusText, Point(15, 35), FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 255), 1);

	//double lamda = ( m.m20 + m.m02)*0.5 +  
	frame_counter++;
	//if (frame_counter % 5)
	//	cout << frame_counter << " : " << theta << endl;//Mat(p1) << endl;
	imshow("Foreground debug", mask);


	return mask_status;
}


int BackSubs::show_forgnd_and_bgnd_init(VideoCapture vidSource_LeftCam)
{
	vidName		= ""; 
	StatusText	= "NAN"; 
	mog			= createBackgroundSubtractorMOG2(BackSubs_History , BackSubs_Threshould , BackSubs_DetectShadows);

	cap = vidSource_LeftCam;
	fps = cap.get(CV_CAP_PROP_FPS);
	if(fps<=0)
		loopWait=33;
	else
		loopWait=1000/fps; // 1000/30=33; 1000/15=67

	namedWindow( "Foreground "		, CV_WINDOW_AUTOSIZE );
	namedWindow( "Foreground debug"	, CV_WINDOW_AUTOSIZE );

	return 0;
}

int BackSubs::show_forgnd(Mat frame)  // assuming input of vreified non-empty frame
{
	/* apply background substraction and manipulate the resultant frame */
	mog->apply(frame,foreground); 
	imshow("BackSubs Foreground before manipulations",foreground); //debugging

    threshold	(foreground,	foreground,	128,	255,THRESH_BINARY);//28,128,198
    medianBlur	(foreground,	foreground,	3);//9
    erode		(foreground,	foreground,	Mat());
    dilate		(foreground,	foreground,	Mat());

	imshow("BackSubs Foreground",foreground); 

	middle_tmp_frame = foreground.clone();
	// ---now 'foreground' it is a workable image binary--- // 
	int frame_status = doMYbsManipulation(middle_tmp_frame);
	BackSubs_State = STANDBY ;
	if (frame_status> 10)
		BackSubs_State = FOUND_SOME_MOVEMENT ;
	else
		if (frame_status> 95)
			BackSubs_State = INITIALIZING ;
	/// thus far the main stuff of BackgroundSubs. from now on just extra manipulations
	//show_more_details(foreground);
	return frame_status;
}

int BackSubs::show_more_details(Mat frame) 
{
	// clear list, copy image
	selected_shapes_contours.clear();
	image	=	frame.clone();

	// find contours and store them all as a list
	middle_tmp_frame	= foreground.clone();
	findContours(middle_tmp_frame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);// See squares.c in the OpenCV sample directory.
	found_contours_num	= contours.size();

	vector<Point> approx;

	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> >	contours_poly	(found_contours_num);
	vector<Rect>			boundRect		(found_contours_num);
	vector<Point2f>			center			(found_contours_num);
	vector<float>			radius			(found_contours_num);
	vector<Moments>			curves_moments	(found_contours_num);
	vector<Point2f>			mass_centers	(found_contours_num); 

	// TODO:  use different resulutions for capture and for analysis.
	//	//TODO:  to gray, and low freq noise reduction
	//gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	//	gray = cv2.GaussianBlur(gray, (21, 21), 0)

	//////////////////////////
//			 test each contour
	StatusText = "";
	if (found_contours_num > 0) {
		StatusText = " sensed some movement ";
	}
	vector<Vec4i> hierarchy;
	for (size_t i = 0; i <found_contours_num ; i++)
	{
		// approximate contour with accuracy proportional to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		boundRect[i] = boundingRect(approx);
		random_color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		if (SHOW_MOVING_CONTOURS)
			//drawContours(image, approx, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());
			drawContours(image, contours, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());
		if (SHOW_MOVING_RECTANGLES)
			rectangle(image,
				boundRect[i].tl(), boundRect[i].br(),
				random_color,		1, 8, 0);
		// contours should have relatively large area (to filter out noisy contours)
		// and be convex.
		// Note: absolute value of an area is used because area may be positive or negative - 
		// in accordance with the contour orientation
			
		area = fabs(contourArea(Mat(contours[i])));
		if (area > MIN_CURVE_AREA && area < MAX_CURVE_AREA && isContourConvex(Mat(contours[i]))) {
			StatusText = "--Major Movement detected--";

			///// .push_back(area);->weighted area->center of movement, and avereged
			selected_shapes_contours	.push_back(contours[i]);///approx
			curves_moments				.push_back(moments(contours[i], false));  // not for display. just for info or other use.
			mass_centers				.push_back(Point2f(	curves_moments[i].m10 / curves_moments[i].m00,
															curves_moments[i].m01 / curves_moments[i].m00));
			if (SHOW_MOVING_BIG_CONTOURS)
				//drawContours(image, approx, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());
				drawContours(image, contours, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());

		}
	}
	// keep history of points.
	// dispay a line of done trajectory in 3D space, and show prediction for planned trajectory.
		
	/* displays and drawings */
	putText(image, StatusText, Point(10, 30), FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2);
	///drawShapesContours(image, selected_shapes_contours, curves_moments, mass_centers);
    //
	imshow( "BackSubs Capture "		,image );
    imshow( "Foreground "	,foreground);

	//TODO:
	//return foreground

	return 0;
}
