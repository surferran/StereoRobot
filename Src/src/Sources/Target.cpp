// file : Target.cpp
 
#include "..\Headers\Target.hpp"
 

// TODO: return parameters of rCircle, boundRect, theta, frame_counter(of bkgSubs) (as part of class?)
// calculate some parameters for the current possable target (frame foreground)
Target::TargetProperties	Target::calc_target_properties(Mat image_mask) 
{ 
	m			= moments(image_mask, false);				// points moment 
	MassCenter	= Point(m.m10/m.m00, m.m01/m.m00);	// mass_centers
	rCircle		= sqrt(m.m00/3.14)/rCircle_devider ;				// estimated rounding circle for the object area
	boundRect	= boundingRect ( image_mask );
	theta		= 0.5 * atan2(2*m.m11, m.m20-m.m02) *   57.3;		//rad 2 deg

	// show in Percent, the relation between bounding rectangle area , and area of the image
	double tmp1		= 100. * boundRect.area() ;
	double tmp2		= (image_mask.size()).area() ; 
	boundAreaRatio	= tmp1 / tmp2;
	 
	int w =  image_mask.size().width;
 
	if (  (2.*rCircle < w* 0.9) && (2.*rCircle > w * 0.1)  		///(MassCenter.x > w/2 - w_band ) && (MassCenter.x < w/2 + w_band )				
		)
	{
		if ( (MassCenter.x > w * 0.3 ) && (MassCenter.x < w * 0.7 ) 
			&& ( boundAreaRatio > 15 )
			)
			return Target_potential_move;
		if ( (MassCenter.x > w * 0.4 ) && (MassCenter.x < w * 0.6 )   
				&& (2.*rCircle < w* 0.5) && (2.*rCircle > w * 0.1)
			)
			return Target_present;		// more centered and stabilized obeject - treated as Good Target	
	} 

	return Target_NA; 
}
