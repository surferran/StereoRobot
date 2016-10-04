// file : Target.cpp
 
#ifdef COMPILING_ON_ROBOT
#include "Target.hpp"
#else
#include "..\Headers\Target.hpp"
#endif


// TODO: return parameters of rCircle, boundRect, theta, frame_counter(of bkgSubs) (as part of class?)
// calculate some parameters for the current possable target (frame foreground)
void Target::calc_target_mask_properties(Mat image_mask) 
{ 
	target_mask_prop.maskIm		=	image_mask;
	m			= moments(image_mask, false);				// points moment 
	target_mask_prop.MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
	target_mask_prop.rCircle	= sqrt(m.m00/3.14)/rCircle_devider ;				// estimated rounding circle for the object area
	target_mask_prop.boundRect	= boundingRect ( image_mask );
	target_mask_prop.theta		= 0.5 * atan2(2*m.m11, m.m20-m.m02) *   57.3;		//rad 2 deg

	// show in Percent, the relation between bounding rectangle area , and area of the image
	double tmp1		= 100. * target_mask_prop.boundRect.area() ;
	double tmp2		= (image_mask.size()).area() ; 
	target_mask_prop.boundAreaRatio	= tmp1 / tmp2;
	 
	target_mask_prop.image_mask_size_width	=  image_mask.size().width;  
}

// TODO: return parameters of rCircle, boundRect, theta, frame_counter(of bkgSubs) (as part of class?)
// calculate some parameters for the current possable target (frame foreground)
Target::TargetState	Target::check_target_mask_properties() 
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

bool	Target::check_target_mask_properties_option1() // option 1 - is for stage __
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
