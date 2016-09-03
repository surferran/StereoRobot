#include "myGUI_handler.h"

myGUI_handler::myGUI_handler()
{

	plotWindowsNames[0] = "win1 - right stereo image";
	plotWindowsNames[1] = "win2 - left stereo image"; 
	plotWindowsNames[2] = "win3 - calculated disparity";
	plotWindowsNames[3] = "win4 - background substruction output";
	plotWindowsNames[4] = "win5 - tracked object";
	plotWindowsNames[5] = "win6 - resultant depth";

}


// shows the 4 images of previous and current images and their matching feature points flow.
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
	//cvShowManyImages("images prev & current" , 4 , im_mat1, im_mat2, im_mat1, im_mat2 );
	cvShowManyImages("images prev & current " , 4 , im_mat1, im_mat2 , im_mat3, im_mat4 );

	if  (1==2)   // TODO: print only for 1st debugging
		cout  << "trackedFeatures size " << trackedFeatures.size() 
			<< " corners size " << corners.size() << " status size " 
			<< status.size() << " status non zeroes "<< countNonZero(status) <<"\n"; 

/////////////////////////
/////////////////////////
}



void myGUI_handler::draw_output_frames(String* WinNames, Mat* images)
{
	for (int i=0; i<thumb_num ; i++)
	{
		imshow( WinNames[i]	, images[i] );
	}
}

// add graphic layer to the image - for showing circle and bounding box for the target tracking.
void myGUI_handler::show_graphics_with_image(Mat & mask, Point MassCenter, double rCircle, Rect boundRect, 
	double theta, double boundAreaRatio, int mask_status)
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


	///frame_counter++;
	//if (frame_counter % 5)
	//	cout << frame_counter << " : " << theta << endl;//Mat(p1) << endl;

	imshow("Foreground debug", mask);
}


/////////////////////////////

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
string myGUI_handler::_sysStatToString(SYSTEM_STATUS val){

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
void myGUI_handler::add_Cross_to_Image(int x, int y, bool addLabel, SYSTEM_STATUS sys_stat, Mat &cameraFeed)   
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

	String StatusText = _sysStatToString(sys_stat);
	putText(cameraFeed, StatusText, Point(15, 15), FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 255), 1);


}

//
void myGUI_handler::display_rectified_pair(Size imageSize , Mat Rimg, Mat Limg, Rect validROI1, Rect validROI2 )
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

	//for( i = 0; i < nimages; i++ )
	{
		Rect validRoi[2] = {validROI1, validROI2};

		for( int k = 0; k < 2; k++ )
		{
			Mat rimg, cimg; 
			//Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
			if (k==1) cimg = Limg;
			else
				cimg = Rimg;
			//remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
		///	cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			cvtColor(cimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = isHorizontalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h)); 
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			//if( useCalibrated )
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
			}
		}
		 
		if (isHorizontalStereo)
			for( int j = 0; j < canvas.rows; j += 16 )
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for( int j = 0; j < canvas.cols; j += 16 )
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
				
		imshow("rectified", canvas);
	}
}