#include "..\Headers\BackgroundSub.hpp"

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


/*************************************************************************************/
/******************************    the class functions section    *********************************/
/*************************************************************************************/

// TODO: return parameters of rCircle, boundRect, theta, frame_counter(of bkgSubs) (as part of class?)
// calculate and print some parameters for the current frame foreground
int BackSubs::doMYbsManipulation( Mat & mask , Point *movementMassCenter)
{ 
	/* inner calculation vars */
	static int	frame_counter=0;
	int			mask_status = 0;
	Moments		m;
	double		boundAreaRatio;

	m			= moments(mask, false);				// points moment 
	MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
	rCircle		= sqrt(m.m00/3.14)/13 ;				// estimated rounding circle for the object area
	boundRect	= boundingRect ( mask );
	theta		= 0.5 * atan2(2*m.m11, m.m20-m.m02) * 57.3;
	// show in Percent, the relation between bounding rectangle area , and area of the image
	double tmp1		= 100. * boundRect.area() ;
	double tmp2		= (mask.size()).area() ; 
	boundAreaRatio	= tmp1 / tmp2;
	mask_status		= int(boundAreaRatio);


	frame_counter++;
	//if (frame_counter % 5) 
	//	cout << frame_counter << " : " << theta << endl;//Mat(p1) << endl;
	


	if ((frame_counter <= 10)  // wait for at least 10 initial frames
		&& (system_state == INITIALIZING))
		return 0;

	if ((frame_counter > 10) //50 // wait for at least 10 initial frames
		&& (system_state == INITIALIZING)
		&& (rCircle < 5) )
		///&& mask.empty() )
	{
		stable_bkgnd_phase	= 1;
		mask_status			= 111;		// will move to STANDBY

		return mask_status;
	}
	if (stable_bkgnd_phase==0)
		return 0;

	myGUI.show_graphics_with_image(mask, MassCenter, rCircle, boundRect, theta, boundAreaRatio, mask_status , frame_counter);

	//TODO: make this condition more clear to read and understand
	int w =  mask.size().width;
	int w_band = 40;
	if ( ///(boundRect.width < w) && 
		(stable_bkgnd_phase==1) 
		&& (2.*rCircle < w* 0.9) && (2.*rCircle > w * 0.1)  		///(MassCenter.x > w/2 - w_band ) && (MassCenter.x < w/2 + w_band )				
	   )
	{
		if ( (MassCenter.x > w * 0.3 ) && (MassCenter.x < w * 0.7 ) 
			 && ( boundAreaRatio > 15 )
		   )
			mask_status = 222;		// treated as Good potential Target
		//if ( (MassCenter.x > w * 0.4 ) && (MassCenter.x < w * 0.6 )   
		//	 && (2.*rCircle < w* 0.5) && (2.*rCircle > w * 0.1)
		//	)
		//	mask_status = 333;		// more centered and stabilized obeject - treated as Good Target	
	} 

	*movementMassCenter = MassCenter;
	return mask_status;
}


int BackSubs::show_forgnd_and_bgnd_init(int fpsIN)
	//VideoCapture vidSource_LeftCam)
{
	//vidName		= ""; 
	StatusText	= "NAN"; 
	mog			= createBackgroundSubtractorMOG2(BackSubs_History , BackSubs_Threshould , BackSubs_DetectShadows);

	//cap = vidSource_LeftCam;
	fps = fpsIN;//cap.get(CV_CAP_PROP_FPS);
	if(fps<=0)
		loopWait=33;
	else
		loopWait=1000/fps; // 1000/30=33; 1000/15=67

	//namedWindow( "Foreground "		, CV_WINDOW_AUTOSIZE );
	namedWindow( "Foreground debug"	, CV_WINDOW_AUTOSIZE );

	return 0;
}

	/* apply background substraction and manipulate the resultant frame */
int BackSubs::find_forgnd(Mat frame, Point *movementMassCenter)  // assuming input of vreified non-empty frame
{

	medianBlur	(frame,	frame,	3); // new

	mog->apply(frame,foreground, BackSubs_LearningRate);	
	///mog->getBackgroundImage(backgroundAvg);
///	imshow("BackSubs Foreground before manipulations",foreground); //debugging
	///imshow("BackSubs background average",backgroundAvg); //debugging

    ///threshold	(foreground,	foreground,	128,	255,THRESH_BINARY);//28,128,198
    ///medianBlur	(foreground,	foreground,	3);//9
	/* open: dst = open( src, element) = dilate( erode( src, element ) ) */
	/* close: dst = close( src, element ) = erode( dilate( src, element ) ) */
	dilate		(foreground,	foreground,	Mat());
    erode		(foreground,	foreground,	Mat());
    ///dilate		(foreground,	foreground,	Mat()); /////
	threshold	(foreground,	foreground,	128,	255,THRESH_BINARY);//28,128,198

///	imshow("BackSubs Foreground",foreground); 

	middle_tmp_frame = foreground.clone();
	// ---now 'foreground' it is a workable image binary--- // 
	int frame_status = doMYbsManipulation(middle_tmp_frame, movementMassCenter);	//also prints. to screen

	if ((system_state == INITIALIZING) && (frame_status==111))
	{
		system_state = STANDBY ; 
		return frame_status;
	}
	if ((system_state == STANDBY) && (frame_status==222))
	{
		system_state = FOUND_SOME_MOVEMENT ; 
		return frame_status;
	}
	if ((system_state == FOUND_SOME_MOVEMENT) && (frame_status==333))
	{
		//TODO: if also disperity is fine ,, then :
			system_state = FOUND_GOOD_TARGET; 
		return frame_status;
	}

	/// thus far the main stuff of BackgroundSubs. from now on just extra manipulations
	//show_more_details(foreground);
	return frame_status;
}
//
//int BackSubs::show_more_details(Mat frame) 
//{
//	// clear list, copy image
//	selected_shapes_contours.clear();
//	image	=	frame.clone();
//
//	// find contours and store them all as a list
//	middle_tmp_frame	= foreground.clone();
//	findContours(middle_tmp_frame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);// See squares.c in the OpenCV sample directory.
//	found_contours_num	= contours.size();
//
//	vector<Point> approx;
//
//	/// Approximate contours to polygons + get bounding rects and circles
//	vector<vector<Point> >	contours_poly	(found_contours_num);
//	vector<Rect>			boundRect		(found_contours_num);
//	vector<Point2f>			center			(found_contours_num);
//	vector<float>			radius			(found_contours_num);
//	vector<Moments>			curves_moments	(found_contours_num);
//	vector<Point2f>			mass_centers	(found_contours_num); 
//
//	// TODO:  use different resulutions for capture and for analysis.
//	//	//TODO:  to gray, and low freq noise reduction
//	//gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
//	//	gray = cv2.GaussianBlur(gray, (21, 21), 0)
//
//	//////////////////////////
////			 test each contour
//	StatusText = "";
//	if (found_contours_num > 0) {
//		StatusText = " sensed some movement ";
//	}
//	vector<Vec4i> hierarchy;
//	for (size_t i = 0; i <found_contours_num ; i++)
//	{
//		// approximate contour with accuracy proportional to the contour perimeter
//		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
//		boundRect[i] = boundingRect(approx);
//		random_color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//		if (SHOW_MOVING_CONTOURS)
//			//drawContours(image, approx, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());
//			drawContours(image, contours, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());
//		if (SHOW_MOVING_RECTANGLES)
//			rectangle(image,
//				boundRect[i].tl(), boundRect[i].br(),
//				random_color,		1, 8, 0);
//		// contours should have relatively large area (to filter out noisy contours)
//		// and be convex.
//		// Note: absolute value of an area is used because area may be positive or negative - 
//		// in accordance with the contour orientation
//			
//		area = fabs(contourArea(Mat(contours[i])));
//		if (area > MIN_CURVE_AREA && area < MAX_CURVE_AREA && isContourConvex(Mat(contours[i]))) {
//			StatusText = "--Major Movement detected--";
//
//			///// .push_back(area);->weighted area->center of movement, and avereged
//			selected_shapes_contours	.push_back(contours[i]);///approx
//			curves_moments				.push_back(moments(contours[i], false));  // not for display. just for info or other use.
//			mass_centers				.push_back(Point2f(	curves_moments[i].m10 / curves_moments[i].m00,
//															curves_moments[i].m01 / curves_moments[i].m00));
//			if (SHOW_MOVING_BIG_CONTOURS)
//				//drawContours(image, approx, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());
//				drawContours(image, contours, i, random_color, 1, 8, vector<Vec4i>(), 0, Point());
//
//		}
//	}
//	// keep history of points.
//	// dispay a line of done trajectory in 3D space, and show prediction for planned trajectory.
//		
//	/* displays and drawings */
//	putText(image, StatusText, Point(10, 30), FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 2);
//	///drawShapesContours(image, selected_shapes_contours, curves_moments, mass_centers);
//    //
//	imshow( "BackSubs Capture "		,image );
//    imshow( "Foreground "	,foreground);
//
//	//TODO:
//	//return foreground
//
//	return 0;
//}
