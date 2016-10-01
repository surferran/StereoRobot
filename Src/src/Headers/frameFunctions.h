/*file : frameFunctions.h
frame function

chnge to hpp

*/
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//
//void process_frame(Mat *inFrame, Mat *outFrame)
//{
//	//rgb_frame_to_Edges(&frame[j], &edges);  // (in, out) Mat order
//	//rgb_frame_reduce_Blue(&frame[j], &edges);  // (in, out) Mat order
//	copy_frame	(inFrame, outFrame);
//	// color_to_gray (..)
//	// smooth image
//	// find edges
//	// check main areas
//	// remove background
//}
//

void copy_frame(Mat *inFrame, Mat *outFrame)
{
	*outFrame = *inFrame;
}

void rgb_frame_to_Edges(Mat *inFrame, Mat *outFrame)
{
	cvtColor(*inFrame, *outFrame, COLOR_BGR2GRAY);
	GaussianBlur(*outFrame, *outFrame, Size(7, 7), 1.5, 1.5);
	Canny(*outFrame, *outFrame, 0, 30, 3);
}

void rgb_frame_reduce_Blue(Mat *inFrame, Mat *outFrame)
{ // tried to copy from Python Code. it doesn't outcome similar..
// need to copy from :
	// http://stackoverflow.com/questions/9018906/detect-rgb-color-interval-with-opencv-and-c 

	//# Convert BGR to HSV
	cvtColor(*inFrame, *outFrame, COLOR_BGR2HSV);  // hsv
	//# define range of blue color in HSV
	//int	lower_blue[3] = { 110,  50,  50 };
	Scalar lower_blue = Scalar( 110, 50, 50,0 );
	//int	upper_blue[3] = { 130, 255, 255 };
	Scalar upper_blue = Scalar(130, 250, 250, 0);
	//# Threshold the HSV image to get only blue colors
	inRange(*inFrame, lower_blue, upper_blue, *outFrame );   //mask

	//# Bitwise - AND mask and original image
	bitwise_and(*outFrame, *outFrame, *outFrame); // res
}


/* 

used tutorials:
 http://docs.opencv.org/3.0-beta/modules/videoio/doc/reading_and_writing_video.html
possible examples to check :
 C:\OpenCV_Source\samples\cpp\tutorial_code\objectDetection

*/

class myWaterShed
{
private:
	Mat originalInput;
	Mat markerMask, 
		imgGray ;

	Mat		img__;
	Point	prevPt;

	bool userVisited = false;

public:
	myWaterShed() 
	{
		prevPt = Point(-1, -1) ; 

		///setMouseCallback( myGUI.plotWindowsNames[1] , &(myGUI.onMouseWSHED), 0 );
	};

	~myWaterShed() {};
	
	void init_mask_by_input(Mat in)
	{
		originalInput	= in;

		cvtColor(in			, markerMask	, COLOR_BGR2GRAY);
		cvtColor(markerMask	, imgGray		, COLOR_GRAY2BGR);	// imgGray - gray duplicated in 3 channels - for display
		markerMask		= Scalar::all(0);

		myGUI.markerMask_WSHED	= markerMask;
		myGUI.img_WSHED			= imgGray;
		myGUI.prevPt_WSHED		= Point(-1, -1);

	};
	
	void calculate_the_watershed(Mat newMask)
	{
		double tt = (double)getTickCount(); 

		int i, j, compCount = 0;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		markerMask = newMask;
		findContours(markerMask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

		if( contours.empty() )
			return;
		Mat markers(markerMask.size(), CV_32S);
		markers = Scalar::all(0);
		int idx = 0;
		for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
			drawContours(markers, contours, idx, Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);

		if( compCount == 0 )
			return;

		vector<Vec3b> colorTab;
		for( i = 0; i < compCount; i++ )
		{
			int b = theRNG().uniform(0, 255);
			int g = theRNG().uniform(0, 255);
			int r = theRNG().uniform(0, 255);

			colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
		}

		double t = (double)getTickCount();
		watershed( originalInput, markers );
		t = (double)getTickCount() - t;
		printf( "execution time = %gms\n", t*1000./getTickFrequency() );

		Mat wshed(markers.size(), CV_8UC3);

		// paint the watershed image
		for( i = 0; i < markers.rows; i++ )
			for( j = 0; j < markers.cols; j++ )
			{
				int index = markers.at<int>(i,j);
				if( index == -1 )
					wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
				else if( index <= 0 || index > compCount )
					wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
				else
					wshed.at<Vec3b>(i,j) = colorTab[index - 1];
			}
		Mat wshed2 = wshed*1.;
		imshow( "watershed segmentation", wshed2 );

		wshed = wshed*0.5 + imgGray*0.5;
		imshow( "watershed transform", wshed );


		tt = (double)getTickCount() - tt;
		printf( "execution time = %gms\n", tt*1000./getTickFrequency() );

	};
};


////////////////////
template <typename T>
class myMatQueue	//possible to set as template for making any type queue 
{
private:
	static const int N = 3 ; //size of queue

	int		nextArrayIndex;

	T		typeArray[N],
			lastElement,
			arrayRunningSum,
			arrayRunningAvg;

	bool	sumInitialized ;

public:

	myMatQueue()
	{
		nextArrayIndex =	0 ;
		sumInitialized	=	false;
	}

	~myMatQueue(){};

	void populateNextElementInArray(Mat currentMat)
	{
		if (!sumInitialized)
		{
			arrayRunningSum	=	Mat::zeros( currentMat.size() , currentMat.type() );
			for (int i=0; i<N; i++)
				typeArray[i] = arrayRunningSum;
			sumInitialized	=	true;
		}
		//TODO: add considaration for the first N elements (divide by smaller then N)
		arrayRunningSum					= arrayRunningSum - typeArray[nextArrayIndex] ;
		typeArray[nextArrayIndex]		= currentMat.clone() ;
		arrayRunningSum					= arrayRunningSum + typeArray[nextArrayIndex] ;
		nextArrayIndex					= (nextArrayIndex+1)%N;	//make it cyclic on 0..(N-1) range
		arrayRunningAvg					= arrayRunningSum / N ;
	}

	void populateNextElementInArray(double currentDbl)
	{
		if (!sumInitialized)
		{
			arrayRunningSum	=	0;
			for (int i=0; i<N; i++)
				typeArray[i] = arrayRunningSum;
			sumInitialized	=	true;
		}
		//TODO: add considaration for the first N elements (divide by smaller then N)
		arrayRunningSum					= arrayRunningSum - typeArray[nextArrayIndex] ;
		typeArray[nextArrayIndex]		= currentDbl ;
		arrayRunningSum					= arrayRunningSum + typeArray[nextArrayIndex] ;
		nextArrayIndex					= (nextArrayIndex+1)%N;	//make it cyclic on 0..(N-1) range
		arrayRunningAvg					= arrayRunningSum / N ;
	}
	void getAvgElement(Mat *avg)
	{
		*avg = arrayRunningAvg.clone() ;
	}

	void getSumElement(Mat *sum)
	{
		*sum = arrayRunningSum.clone() ;
	}

	void getAvgElement(double *avg)
	{
		*avg = arrayRunningAvg  ;
	}

	void getSumElement(double *sum)
	{
		*sum = arrayRunningSum ;
	}
	
};