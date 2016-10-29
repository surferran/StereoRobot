#include "BackgroundSub.hpp"
#include "StereoRobotApp.hpp"
#include "myGUI_handler.h"

RNG						rng(12345);
extern myGUI_handler	myGUI;

/*************************************************************************************/
/******************************    the class functions section    *********************************/
/*************************************************************************************/

// TODO: move this funtion to Target module. set and check . !
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

	double c2r		=	100.*m.m00 /255.0 / boundRect.area() ;	// circle area , divided by, rect.area // circle to rectangle

	myGUI.show_BgSubt(mask, MassCenter, rCircle, boundRect, theta, boundAreaRatio, c2r/*mask_status*/ , frame_counter);

	int rCircleFromLost_factor = 1;
	if  (BgSubt_Status == BG_RECOVER_FROM_LOST) ///will pass over the INIT phase
	{ 
		/* up to 10 cycles from lost - be more flexible with conditions */
		rCircleFromLost_factor = 7./6. * (10- cycles_number_after_lost) ; 
		
		if ( rCircleFromLost_factor < 1 ) // end this conditioning
		{ 
			rCircleFromLost_factor		= 1;
			cycles_number_after_lost	= 0;
			stable_bkgnd_phase			= 1;
			BgSubt_Status				= BG_STANDING_BY;
		}
		if ( rCircle < 5 * rCircleFromLost_factor )	
		{
			stable_bkgnd_phase	= 1;
			BgSubt_Status		= BG_STANDING_BY;
		}
	}
	else
	if ( (BgSubt_Status == BG_INITIALIZING) && 
		  (( rCircle < 5 ) || (frame_counter > 10)) ) 
	{ 
		stable_bkgnd_phase	= 1;
		BgSubt_Status		= BG_STANDING_BY;
		return mask_status;
	}

	if (stable_bkgnd_phase==0)
		return 0;

	/* check condition for good potential target */
	int w =  mask.size().width;

	double w_ratio_diff = 0.05;///0.2 // 0.05 is the must minimum

	if ( ///(c2r > 22) &&
		(stable_bkgnd_phase==1) 
		&& (2.*rCircle < w* 0.9) && (2.*rCircle > w * 0.1)  			
	   )
	{
		if ( (MassCenter.x > w * (w_ratio_diff) ) && (MassCenter.x < w * (1 - w_ratio_diff) ) 
				&& ( boundAreaRatio > 5 )	&& ( boundAreaRatio < 95 )	//15
		   )
			BgSubt_Status = BG_FOUND_MOVEMENT	;
	} 

	*movementMassCenter = MassCenter;
	return mask_status;
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

BackSubs::BackSubs()
{
	BgSubt_Status = BG_INITIALIZING ;
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

	BgSubt_Status		=	BG_INITIALIZING ;
	frame_counter		=	0;
	init_frames_number	=	2;	 
	cycles_number_after_lost	= 0;
	if (lostFlag) {
		cycles_number_after_lost++;
		BgSubt_Status	=	BG_RECOVER_FROM_LOST;
	}
	foreground = Mat();

	return 0;
}

	/* apply background subtraction and manipulate the resultant frame */
void BackSubs::find_forgnd(Mat frame, Point *movementMassCenter)  
{
	Mat bgSubIn = frame.clone();

	medianBlur	(bgSubIn,	bgSubIn,	3); 
	mog->apply(bgSubIn,foreground, BackSubs_LearningRate);	

	frame_counter++;
	if (BgSubt_Status == BG_RECOVER_FROM_LOST)	
		cycles_number_after_lost++;
 
	/* wait for at least # initial frames, after reset or lost */
	if (frame_counter < init_frames_number)
		return ;

	/* start manipulating and checking result */
	/* open:  dst = open( src, element)   = dilate( erode( src, element ) ) */
	/* close: dst = close( src, element ) = erode( dilate( src, element ) ) */
	dilate		(foreground,	foreground,	Mat());
    erode		(foreground,	foreground,	Mat()); 
	threshold	(foreground,	foreground,	128,	255,THRESH_BINARY);  

	middle_tmp_frame = foreground.clone();

#ifndef COMPILING_ON_ROBOT
	myGUI.showContours(middle_tmp_frame);
#endif

	checkBgSubt_MaskResult(middle_tmp_frame, movementMassCenter);	//also prints. to screen
}
