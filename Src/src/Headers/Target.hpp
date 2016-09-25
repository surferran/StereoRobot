// file : Target.hpp

#pragma once
 
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/features2d/features2d.hpp"
#include <stdio.h>
 
using namespace cv;
 

/*************************************************************************************/
/******************************    Header section    *********************************/
/*************************************************************************************/
class Target
{
public:

	Target(){
		inner_target_phases=0;
	};

	 enum TargetProperties {
		Target_NA,
		Target_potential_move,
		Target_present
	 };

	 TargetProperties		calc_target_properties(Mat image_mask); //BRG or Gray?

	/* foreground movement features */
	Point	MassCenter ;
	double	rCircle;			// estimated rounding circle for the object area
	Rect	boundRect;
	double	theta ;				// estimated oriantation of bounding box. though not well feature
	double	boundAreaRatio;

private:
	 
	Moments m;

	// recommanded factors:
	double	rCircle_devider		=	13.0 ;
	//0.9
	//	0.1
	//	0.3
	//	0.7
	//	0.4
	//	0.6

	//if (  (2.*rCircle < w* 0.9) && (2.*rCircle > w * 0.1)  		///(MassCenter.x > w/2 - w_band ) && (MassCenter.x < w/2 + w_band )				
	//	)
	//{
	//	if ( (MassCenter.x > w * 0.3 ) && (MassCenter.x < w * 0.7 ) 
	//		&& ( boundAreaRatio > 15 )
	//		)
	//		return Target_potential_move;
	//	if ( (MassCenter.x > w * 0.4 ) && (MassCenter.x < w * 0.6 )   
	//		&& (2.*rCircle < w* 0.5) && (2.*rCircle > w * 0.1)

	int inner_target_phases ;	//0-init, 1-possible_move, 2-fit to good target, 3-was,but lost now.
	 	
};
/*************************************************************************************/
/******************************end of Header section *********************************/
/*************************************************************************************/
 