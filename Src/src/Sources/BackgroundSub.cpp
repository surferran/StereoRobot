///#ifdef COMPILING_ON_ROBOT
#include "BackgroundSub.hpp"
//#else
//#include "..\Headers\BackgroundSub.hpp"
//#endif

#include "..\Headers\StereoRobotApp.hpp"
#include "myGUI_handler.h"

RNG						rng(12345);
extern myGUI_handler	myGUI;

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
		&& (myCApp.system_state == StereoRobotApp::INITIALIZING))
		return 0;

	if ((frame_counter > 10) //50 // wait for at least 10 initial frames
		&& (myCApp.system_state == StereoRobotApp::INITIALIZING)
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
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

BackSubs::BackSubs()
{
	BgSubt_Status = INITIALIZING ;
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

	namedWindow( "Foreground debug"	, CV_WINDOW_AUTOSIZE );

	return 0;
}

	/* apply background substraction and manipulate the resultant frame */
void BackSubs::find_forgnd(Mat frame, Point *movementMassCenter)  // assuming input of vreified non-empty frame
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

	if ((myCApp.system_state == StereoRobotApp::INITIALIZING) && (frame_status==111))
	{
		//system_state = StereoRobotApp::STANDBY ; 
		BgSubt_Status = STANDING_BY ;
		//return frame_status;
	}
	if ((myCApp.system_state == StereoRobotApp::STANDBY) && (frame_status==222))
	{
		//system_state = StereoRobotApp::FOUND_SOME_MOVEMENT ; 
		BgSubt_Status = FOUND_MOVEMENT ;
		//return frame_status;
	}
}