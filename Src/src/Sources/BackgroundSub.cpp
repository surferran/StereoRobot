#include "BackgroundSub.hpp"
#include "StereoRobotApp.hpp"
#include "myGUI_handler.h"

RNG						rng(12345);
extern myGUI_handler	myGUI;

/*************************************************************************************/
/******************************    the class functions section    *********************************/
/*************************************************************************************/

// TODO: move this funtion to Target module. set and check .
int BackSubs::checkBgSubt_MaskResult( Mat & mask , Point *movementMassCenter)
{ 
	/* inner calculation vars */ 
	int			mask_status = 0;
	Moments		m;
	double		boundAreaRatio;

	m			= moments(mask, false);				// points moment 
	MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
	rCircle		=	sqrt(m.m00 / CV_PI / 255/*rCircle_devider*/) ;// estimated rounding circle for the object area 
	boundRect	= boundingRect ( mask );
	theta		= 0.5 * atan2(2*m.m11, m.m20-m.m02) * 57.3;
	// show in Percent, the relation between bounding rectangle area , and area of the image
	double tmp1		= 100. * boundRect.area() ;
	double tmp2		= (mask.size()).area() ; 
	boundAreaRatio	= tmp1 / tmp2;
	mask_status		= int(boundAreaRatio);

	int rCircleFromLost_factor = 1;
	if  (BgSubt_Status == RECOVER_FROM_LOST) ///will pass over the INIT phase
	{ 
		/* up to 10 cycles from lost - be more flexible with conditions */
		rCircleFromLost_factor = 7./6. * (10- cycles_number_after_lost) ; 
		
		if ( rCircleFromLost_factor < 1 ) // end this conditioning
		{ 
			rCircleFromLost_factor		= 1;
			cycles_number_after_lost	= 0;
			stable_bkgnd_phase			= 1;
			BgSubt_Status				= STANDING_BY;
		}
		if ( rCircle < 5 * rCircleFromLost_factor )	
		{
			stable_bkgnd_phase	= 1;
			BgSubt_Status		= STANDING_BY;
		}
	}
	else
	if ( (BgSubt_Status == StereoRobotApp::INITIALIZING) && ( rCircle < 5 ) ) 
	{ 
		stable_bkgnd_phase	= 1;
		BgSubt_Status		= STANDING_BY;
		return mask_status;
	}

	if (stable_bkgnd_phase==0)
		return 0;

	myGUI.show_BgSubt(mask, MassCenter, rCircle, boundRect, theta, boundAreaRatio, mask_status , frame_counter);

	//TODO: make this condition more clear to read and understand
	int w =  mask.size().width;
	int w_band = 40;
	if ( ///(boundRect.width < w) && 
		(stable_bkgnd_phase==1) 
		&& (2.*rCircle < w* 0.9) && (2.*rCircle > w * 0.1)  			
	   )
	{
		if ( (MassCenter.x > w * 0.2 ) && (MassCenter.x < w * 0.8 ) 
			 && ( boundAreaRatio > 10 )	//15
		   )
			BgSubt_Status = FOUND_MOVEMENT	;
			//mask_status = 222;		// treated as Good potential Target
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

int BackSubs::show_forgnd_and_bgnd_init(int fpsIN, bool lostFlag)
{
	StatusText	= "NAN"; 
	mog			= createBackgroundSubtractorMOG2(BackSubs_History , BackSubs_Threshould , BackSubs_DetectShadows);
	stable_bkgnd_phase = 0;

	if (fpsIN>0)
		fps = fpsIN;
	if(fps<=0)
		loopWait=33;
	else
		loopWait=1000/fps; // 1000/30=33; 1000/15=67

	BgSubt_Status		=	INITIALIZING ;
	frame_counter		=	0;
	init_frames_number	=	2;	 
	cycles_number_after_lost	= 0;
	if (lostFlag) {
		cycles_number_after_lost++;
		BgSubt_Status	=	RECOVER_FROM_LOST;
	}

	return 0;
}

	/* apply background subtraction and manipulate the resultant frame */
void BackSubs::find_forgnd(Mat frame, Point *movementMassCenter)  
{
	Mat bgSubIn = frame.clone();

	medianBlur	(bgSubIn,	bgSubIn,	3); 
	mog->apply(bgSubIn,foreground, BackSubs_LearningRate);	

	frame_counter++;
	if (BgSubt_Status == RECOVER_FROM_LOST)	
		cycles_number_after_lost++;
 
	/* wait for at least # initial frames, after reset or lost */
	if (frame_counter <= init_frames_number)  return ;

	/* start manipulating and checking result */
	/* open:  dst = open( src, element)   = dilate( erode( src, element ) ) */
	/* close: dst = close( src, element ) = erode( dilate( src, element ) ) */
	dilate		(foreground,	foreground,	Mat());
    erode		(foreground,	foreground,	Mat()); 
	threshold	(foreground,	foreground,	128,	255,THRESH_BINARY);  

	middle_tmp_frame = foreground.clone();

	checkBgSubt_MaskResult(middle_tmp_frame, movementMassCenter);	//also prints. to screen

}