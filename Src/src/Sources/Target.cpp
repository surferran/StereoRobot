// file : Target.cpp
 
#ifdef COMPILING_ON_ROBOT
#include "Target.hpp"
#else
#include "..\Headers\Target.hpp"
#endif


// calculate some parameters for the current target mask
void Target::set_target_mask_properties(Mat image_mask) 
{ 
	rFullRef		=	sqrt( image_mask.size().area() / CV_PI );
	minMaxLoc(image_mask, 0, &maxMat);
	///rCircle_devider =	image_mask.size().area() * maxMat;
	rCircle_devider =	 maxMat;
	target_mask_prop.maskIm					=	image_mask;
	target_mask_prop.image_mask_area		=	image_mask.size().area() ;
	target_mask_prop.image_mask_size_width	=	image_mask.size().width  ;  
	target_mask_prop.image_mask_size_height	=	image_mask.size().height ;  
	target_mask_prop.boundRect				=	boundingRect ( image_mask );
	double dbgCheck = sqrt( target_mask_prop.boundRect.area() / CV_PI ) ;

	m			= moments(image_mask, false);				// points moment 
	target_mask_prop.MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
	target_mask_prop.rCircle	= sqrt(m.m00 / CV_PI / rCircle_devider) ;	// estimated rounding circle for the object (area=pi*r^2 -> r=(area_val/pi/maxVal)^0.5)
	target_mask_prop.theta		= 0.5 * atan2(2*m.m11, m.m20-m.m02) *   57.3;		//rad 2 deg    // ratio between x part to y part?!


	// show in Percent, the relation between bounding rectangle area , and area of the image
	double tmp1		= target_mask_prop.boundRect.area() ;
	target_mask_prop.boundAreaRatio	= 100. * tmp1 / target_mask_prop.image_mask_area;
	 
}

Target::TargetState	Target::check_target_mask_properties() //for which case?
{  
	Mat tmp4debug = target_mask_prop.maskIm ;

	int w = target_mask_prop.image_mask_size_width;
	if (  (2.*target_mask_prop.rCircle < w * 0.9) && (2.*target_mask_prop.rCircle > w * 0.1)  		///(MassCenter.x > w/2 - w_band ) && (MassCenter.x < w/2 + w_band )				
		)
	{
		if ( (target_mask_prop.MassCenter.x > w * 0.3 ) && (target_mask_prop.MassCenter.x < w * 0.7 ) 
			&& ( target_mask_prop.boundAreaRatio > 15 )
			)
			return Target_potential_move;
		if ( (target_mask_prop.MassCenter.x > w * 0.4 ) && (target_mask_prop.MassCenter.x < w * 0.6 )   
			&& (2.*target_mask_prop.rCircle < w* 0.5) && (2.*target_mask_prop.rCircle > w * 0.1)
			)
			return Target_present;		// more centered and stabilized obeject - treated as Good Target	
	} 

	return Target_NA; 
}

bool	Target::check_target_mask_properties_for_good_initial_target() // option 1 - is for stage __
{  
	int w = target_mask_prop.image_mask_size_width;
	int area = target_mask_prop.boundAreaRatio;
	if (  (2.*target_mask_prop.rCircle < w * 0.9) && (2.*target_mask_prop.rCircle > w * 0.1)  		///(MassCenter.x > w/2 - w_band ) && (MassCenter.x < w/2 + w_band )				
		)
	{
		if ( (target_mask_prop.MassCenter.x > w * 0.3 ) && (target_mask_prop.MassCenter.x < w * 0.7 ) 
			&& ( target_mask_prop.boundAreaRatio > 15 )
			)
			return true;
		if ( (target_mask_prop.MassCenter.x > w * 0.4 ) && (target_mask_prop.MassCenter.x < w * 0.6 )   
			&& (2.*target_mask_prop.rCircle < w* 0.5) && (2.*target_mask_prop.rCircle > w * 0.1)
			)
			return true;		// more centered and stabilized obeject - treated as Good Target	
	} 

	return false; 
}
