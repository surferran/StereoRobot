// file : Target.hpp
// class for containing target properties and validation functions for each phase or stage.
// the target is checked by the mask of it (binary description) 

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
		inner_target_phases	=	0;
		target_object_prop.relevant_disparity	=	-999;
	};

	 enum TargetState {
		Target_NA,
		Target_potential_move,
		Target_present
	 };

	void			set_target_mask_properties(Mat image_mask); //Gray UINT8
	TargetState		check_target_mask_properties(); 
	bool			check_target_mask_properties_for_good_initial_target(); 

	/* foreground movement features */
	struct target_mask_properties{
		Mat		maskIm;				// the mask itself
		int		image_mask_size_width;
		int		image_mask_size_height;
		int		image_mask_area;
		Point	MassCenter ;		// (x,y) [pix]
		double	rCircle;			// estimated rounding circle for the object area
		Rect	boundRect;
		double	theta ;				// estimated oriantation of bounding box. though not well feature
		double	boundAreaRatio;		// boundRect.area/image.area
	};
	target_mask_properties	target_mask_prop;

	/* target object center of mass (CM) properties */
	struct target_object_properties
	{
		int		relevant_disparity;			//  [pix]
		int		target_estimated_distance ; //	depth [cm]
		int		target_estimated_dx;		//	~bearing [pix]
	};
	target_object_properties target_object_prop;

	Mat		potential_target;

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

	int		inner_target_phases ;	//0-init, 1-possible_move, 2-fit to good target, 3-was,but lost now.
	double	maxMat; 	
	double  rFullRef;
};
/*************************************************************************************/
/******************************end of Header section *********************************/
/*************************************************************************************/
 