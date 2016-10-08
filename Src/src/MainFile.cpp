//// 3D stero robot (RoboDog)
/*
	history can be vied in the GitHub site repository.
	written app : by Ran , year 2016
*/

/*********  main application constants object  ********/
#include "StereoRobotApp.hpp"
StereoRobotApp		myCApp; 

extern ImagesSourceHandler myStereoCams; 

////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////					  main	   			  //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////// 

int main(int argc, char** argv) 
{
	myCApp.appInitializations();

	// part just for easier debug when reading from avi file.
	///myStereoCams.setStartingFrame( 250 );

	myCApp.appMainLoop();

	return 0;

}
