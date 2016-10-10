
#ifdef COMPILING_ON_ROBOT
#include "myGUI_handler.h"
#else
#include "..\Headers\myGUI_handler.h"
#endif

myGUI_handler::myGUI_handler()
{

	plotWindowsNames[WIN1_NDX_RightRawIm]		= "win1 - original Right image";
	plotWindowsNames[WIN2_NDX_LeftRawIm]		= "win2 - original Left image"; 

	plotWindowsNames[WIN3_NDX_BgSubtMask]		= "win3 - background substruction output";
	plotWindowsNames[WIN4_NDX_DisparityMask]	= "win4 - filtered disperity";

	plotWindowsNames[WIN5_NDX_FeaturePoints]	= "win5 - selected feature points";

	//plotWindowsNames[5] = "win6 - depth mask";
	//plotWindowsNames[5] = "win6 - target aquired Depth";
	//plotWindowsNames[6] = "win7 - target aquired Depth Masked";

	//plotWindowsNames[8] = "win9 - potential learned target";

}


void myGUI_handler::show_raw_captures(Mat L_in, Mat R_in, long frameCounter, StereoRobotApp::SYSTEM_STATUS sys_stat )	//inputs in colour
{
	/// show raw images first
	Mat tmpLeft = L_in.clone();
	add_counterFrame(tmpLeft , &frameCounter ) ;
	add_sysStatus_to_Image(sys_stat, tmpLeft);
	imshow(plotWindowsNames[WIN1_NDX_RightRawIm],	R_in	);
	imshow(plotWindowsNames[WIN2_NDX_LeftRawIm],	tmpLeft );
}

void myGUI_handler::show_disparity_map(Mat sum_of_N_disparities, int avg_depth)
{
	Mat tmpIm = sum_of_N_disparities.clone();
	add_distance_to_disparityIM(avg_depth, tmpIm);

	imshow ( plotWindowsNames[WIN4_NDX_DisparityMask], tmpIm );	 

}

void myGUI_handler::close_BgSubt_win()
{
	destroyWindow( plotWindowsNames[WIN3_NDX_BgSubtMask] );
}
void myGUI_handler::close_Tracking_win()
{
	destroyWindow( plotWindowsNames[WIN4_NDX_DisparityMask] );
	destroyWindow( plotWindowsNames[WIN5_NDX_FeaturePoints] ); 
}

// shows the 4 images of previous and current images and 
//						their matching feature points flow.
void myGUI_handler::dispFlowChanges(Mat & prevGrayROI, vector<Point2f> trackedFeatures, Mat& grayROI, vector<Point2f> corners,
										vector<uchar> status)
{
#define MAKE_RESIZE true
	Mat copyPrev; 
	Mat copyCurrent;
	cvtColor(prevGrayROI.clone(),copyPrev	,CV_GRAY2BGR);
	cvtColor(grayROI.clone()	,copyCurrent,CV_GRAY2BGR);
#ifdef MAKE_RESIZE
	resize(copyPrev		, copyPrev		, Size(), 0.5, 0.5, CV_INTER_AREA);  //resize by half
	resize(copyCurrent	, copyCurrent	, Size(), 0.5, 0.5, CV_INTER_AREA);  //resize by half
#endif // MAKE_RESIZE

///////////////////////// 
	int r	= 2;	//3
	for( int i = 0; i < trackedFeatures.size(); i++ )
	{ 
		circle( copyPrev, trackedFeatures[i], r, 
			///Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); 
			Scalar(10, 100, 255), -1, 8, 0 ); 
	}			
	Mat copyCurrent2=copyCurrent.clone() ;
	Mat copyCurrent3=copyCurrent.clone() ;
	//int r	= 2;//3
	for( int i = 0; i < corners.size(); i++ )
	{ 
		if (status[i])
			circle( copyCurrent, corners[i], r,  
				Scalar(0, 255, 0), -1, 8, 0 ); 
		else
			circle( copyCurrent3, corners[i], r+2,  
				Scalar(0, 0, 255), -1, 8, 0 ); 
		circle( copyCurrent2, corners[i], r,  
			Scalar(10, 100, 255), -1, 8, 0 ); 
	}	

	IplImage	im_mat1_ = (IplImage)copyPrev,
				im_mat2_ = (IplImage)copyCurrent,
				im_mat3_ = (IplImage)copyCurrent2,
				im_mat4_ = (IplImage)copyCurrent3;

	IplImage	*im_mat1 = cvCloneImage(&im_mat1_),
				*im_mat2 = cvCloneImage(&im_mat2_),
				*im_mat3 = cvCloneImage(&im_mat3_),
				*im_mat4 = cvCloneImage(&im_mat4_);

///	cvShowManyImages("images prev & current " , 4 , im_mat1, im_mat2 , im_mat3, im_mat4 );

	if  (1==2)   // TODO: print only for 1st debugging
		cout  << "trackedFeatures size " << trackedFeatures.size() 
			<< " corners size " << corners.size() << " status size " 
			<< status.size() << " status non zeroes "<< countNonZero(status) <<"\n"; 

/////////////////////////
/////////////////////////
}


// add graphic layer to the image - for showing circle and bounding box for the target tracking.
void myGUI_handler::show_BgSubt(Mat & mask, Point MassCenter, double rCircle, Rect boundRect, 
	double theta, double boundAreaRatio, int mask_status, int frame_counter)	//TODO: add called_id to indicate in which window to dispay result
{
	String		StatusText ="";

	circle(mask, MassCenter, rCircle, Scalar(128,220,220), 3); 

	rectangle(mask,
		boundRect.tl(), boundRect.br(),
		Scalar(128,220,220) ,	2, 8, 0);

	StatusText = "theta=" + _doubleToString(theta);
	putText(mask, StatusText, Point(15, 15), FONT_HERSHEY_COMPLEX, 0.4, (210, 110, 220), 1);
	StatusText = "rCircle=" + _doubleToString(rCircle);
	putText(mask, StatusText, Point(15, 25), FONT_HERSHEY_COMPLEX, 0.4, (110, 210, 220), 1);

	StatusText  = "boundAR=" + _doubleToString(boundAreaRatio);
	putText(mask, StatusText, Point(15, 35), FONT_HERSHEY_COMPLEX, 0.4, (210, 210, 220), 1);
	StatusText  = "status=" + _intToString(mask_status);
	putText(mask, StatusText, Point(15, 45), FONT_HERSHEY_COMPLEX, 0.4, (210, 210, 220), 1);
	StatusText  = "frame_counter= " + _intToString(frame_counter);
	putText(mask, StatusText, Point(15, 65), FONT_HERSHEY_COMPLEX, 0.4, (210, 210, 220), 1);

	imshow( plotWindowsNames[WIN3_NDX_BgSubtMask] , mask);
}

void myGUI_handler::draw_output_frames(String* WinNames, Mat* images)
{
	for (int i=0; i < NUM_OF_GUI_WINDOWS; i++)	//<= thumb_num ?
	{
		imshow( WinNames[i]	, images[i] );
	}
}
/////////////////////////////

//int to string helper function
string myGUI_handler::_longToString(long number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}
//int to string helper function
string myGUI_handler::_intToString(int number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}
//double to string helper function
string myGUI_handler::_doubleToString(double number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//system status enum to string helper function
string myGUI_handler::_sysStatToString(StereoRobotApp::SYSTEM_STATUS val){

	std::stringstream ss;
	ss<<"";
	switch (val)
	{
	case StereoRobotApp::INITIALIZING:
		ss << "sys : INITIALIZING";
		break;
	case StereoRobotApp::STANDBY:
		ss << "sys : STANDBY";
		break;
	case StereoRobotApp::FOUND_SOME_MOVEMENT:
		ss << "sys : MOVEMENT";
		break;
	case StereoRobotApp::TRACKING:
		ss << "sys : GOOD TARGET";
		break;
	case StereoRobotApp::TARGET_IS_LOST:
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
void myGUI_handler::add_Cross_to_Image(int x, int y, bool addLabel, StereoRobotApp::SYSTEM_STATUS sys_stat, Mat &cameraFeed)   
{
	Scalar	color	=	Scalar(0,255,0) ;
	int		xMid	=	cameraFeed.size().width/2  , 
			yMid	=	cameraFeed.size().height/2 ;

	switch (sys_stat)
	{
	case StereoRobotApp::INITIALIZING:
		color = Scalar(100	, 55	, 10) ;		break;
	case StereoRobotApp::STANDBY:
		color = Scalar(100	, 155	, 110) ;	break;
	case StereoRobotApp::FOUND_SOME_MOVEMENT:
		color = Scalar(100	, 255	, 10) ;		break;
	case StereoRobotApp::TRACKING:
		color = Scalar(1	, 255	, 1) ;		break;
	default:
		color = Scalar(10	, 10	, 10) ;
		break;
	}   

	/* draw circle with crossing lines */
	circle	(cameraFeed,Point(x,y), 10			,color,2); //r20
	line	(cameraFeed,Point(x,y+25),Point(x,y-25),color,2);
	line	(cameraFeed,Point(x+25,y),Point(x-25,y),color,2);
	// line to image center
	line	(cameraFeed,Point(x,y),Point(xMid , yMid),color/2,2);
	/* draw the projections */
	color		= Scalar(10	, 10	, 10) ;
	line	(cameraFeed,Point(x, yMid),Point(xMid , yMid), color, 1, LINE_4);
	line	(cameraFeed,Point(xMid, y),Point(xMid , yMid), color, 1, LINE_4);


	// TODO: consider print it in the image corner. in status bar for example.
	//write the position of the object to the screen
	///putText(cameraFeed,"Tracking object at (" + _intToString(x)+","+_intToString(y)+")",Point(x,y),1,0.51,Scalar(255,0,0),1);
	if (addLabel)
		putText(cameraFeed,"Tracking object at (" + _intToString(x)+","+_intToString(y)+")",
			Point(x/20+1,y/20+20),1,0.51,Scalar(255,0,0),1);
	//alpha is..
	
}
void myGUI_handler::add_sysStatus_to_Image(StereoRobotApp::SYSTEM_STATUS sys_stat, Mat &cameraFeed) 
{
	String StatusText = _sysStatToString(sys_stat);
	putText(cameraFeed, StatusText, Point(15, 15), FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 255), 1);

}

/*
add distance text to disparity image
*/
void myGUI_handler::add_distance_to_disparityIM(double dist, Mat &ImFeed)
{
	Scalar	color	=	Scalar(110,255,0) ;
	int		xMid	=	(int) ((ImFeed).size().width  * 0.65)  , 
			yMid	=	(int) ((ImFeed).size().height * 0.9) ;
	
	dist = (int)dist;
	putText(ImFeed,"" + _doubleToString(dist)+" [cm]",
			Point(xMid, yMid),1,0.85,Scalar(255,0,0),1);
}
//
void myGUI_handler::display_rectified_pair(Size imageSize , Mat Rimg, Mat Limg, Rect validROI1, Rect validROI2 , long FrameCtr)
{
	Mat		canvas;
	double	sf;
	int		w, h;
	bool isHorizontalStereo = true;
	 
	if (isHorizontalStereo)
	{
		sf = 600./MAX(imageSize.width, imageSize.height);
		sf=1;
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w*2, CV_8UC3);
	}
	else
	{
		sf	= 300./MAX(imageSize.width , imageSize.height);
		w	= cvRound(imageSize.width	*	sf);
		h	= cvRound(imageSize.height	*	sf);
		canvas.create(h*2, w, CV_8UC3);
	}

	Rect validRoi[2] = {validROI1, validROI2};

		for( int k = 0; k < 2; k++ )
	{
		Mat rimg, cimg;  
		if (k==1) cimg = Limg;
		else
			cimg = Rimg; 
		cvtColor(cimg, cimg, COLOR_GRAY2BGR);
		Mat canvasPart = isHorizontalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h)); 
		resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA); 
		Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
			cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
		rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
	}
		 
	if (isHorizontalStereo)
		for( int j = 0; j < canvas.rows; j += 16 )
			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	else
		for( int j = 0; j < canvas.cols; j += 16 )
			line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
				
	add_counterFrame(canvas, &FrameCtr);

	imshow("rectified", canvas);	
}


//////////// functions from watershed example ////////////////////


///static 
void myGUI_handler::onMouseWSHED( int event, int x, int y, int flags, void* )
{
	// using
	//Mat markerMask, img__;
	//Point prevPt(-1, -1);

	if( x < 0 || x >= img_WSHED.cols || y < 0 || y >= img_WSHED.rows )
		return;
	if( event == EVENT_LBUTTONUP || !(flags & EVENT_FLAG_LBUTTON) )
		prevPt_WSHED = Point(-1,-1);
	else if( event == EVENT_LBUTTONDOWN )
		prevPt_WSHED = Point(x,y);
	else if( event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON) )
	{
		Point pt(x, y);
		if( prevPt_WSHED.x < 0 )
			prevPt_WSHED = pt;
		line( markerMask_WSHED,	prevPt_WSHED, pt, Scalar::all(255), 5, 8, 0 );
		line( img_WSHED,		prevPt_WSHED, pt, Scalar::all(255), 5, 8, 0 );
		prevPt_WSHED = pt;
		imshow("image", img_WSHED);
	}
}

////////////backup functions////////////////////

int findBiggestContour(vector<vector<Point> > contours){
	int indexOfBiggestContour = -1;
	int sizeOfBiggestContour = 0;
	for (int i = 0; i < contours.size(); i++){
		if(contours[i].size() > sizeOfBiggestContour){
			sizeOfBiggestContour = contours[i].size();
			indexOfBiggestContour = i;
		}
	}
	return indexOfBiggestContour;
}
void makeContours(Mat aBw){

	vector<vector<Point>>	contours;
	vector<vector<int>>		hullI;
	vector<vector<Point>>	hullP;
	vector<vector<Vec4i>>	defects;	
	int						cIdx;
	Rect					bRect;

	findContours(aBw, contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	hullI	=vector<vector<int> >	(contours.size());
	hullP	=vector<vector<Point> >	(contours.size());
	defects	=vector<vector<Vec4i> > (contours.size());
	cIdx	=findBiggestContour( contours);
	if( cIdx!=-1){ 
		bRect=boundingRect(Mat( contours[ cIdx]));		
		convexHull(Mat( contours[ cIdx]), hullP[ cIdx],false,true);
		convexHull(Mat( contours[ cIdx]), hullI[ cIdx],false,false);
		approxPolyDP( Mat( hullP[ cIdx]),  hullP[ cIdx], 18, true );
		///if( contours[ cIdx].size()>3 )
		{
			///	convexityDefects( contours[ cIdx], hullI[ cIdx], defects[ cIdx]);
			///eleminateDefects(m);
		} 
		Scalar color;

		//color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		color = Scalar (2,2,2);
		drawContours(aBw, contours, cIdx , color );

		color = Scalar (2,2,2);
		//color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours(aBw, hullP   , cIdx , color );

		rectangle(aBw,
			bRect.tl(), bRect.br(),
			Scalar(128,220,220) ,	2, 8, 0);

		imshow("hull results", aBw);
	}
}

void myGUI_handler::add_counterFrame(Mat &inImage, long * frameNum)
{
	putText(inImage,"input frame counter (" + _longToString(*frameNum)+")",
		Point(15, inImage.size().height - 15 ),1,0.75,Scalar(155,0,0),2);
}

////////////////////////////