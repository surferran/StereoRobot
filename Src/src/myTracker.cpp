#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>

using namespace cv;
using namespace std;


class Tracker {
private:
    vector<Point2f> trackedFeatures; 
    Mat             prevGrayROI;

public:

	int				TrackPercent = 0;		// % of the feature points that has correspondance to the next/previous frame
    bool            freshStart;
    Mat_<float>     rigidTransform;
	int				min_features				= 40;//200; 
	int				num_of_maxCornersFeatures	= 300;//300;
	float			mid_level_percent			= 1.25;		//	1/1.25=80%

	int		alphaSlider = 0;
	int		alphaSlider2 = 0;
	char	TrackbarName[50]="StatusSum";
	char	TrackbarName2[50]="StatusSumPercent";

	int		alphaSlider_max=100;

    Tracker():freshStart(true) {
        rigidTransform = Mat::eye(3,3,CV_32FC1); //affine 2x3 in a 3x3 matrix
    }

	// both imgTarget, imgROI should be of the same size.
    void processImage(Mat& imgTarget, Mat&imgROI , SYSTEM_STATUS external_state) 
	{

		RNG rng(12345);//RANDV

		Mat grayTarget; cvtColor(imgTarget,grayTarget,CV_BGR2GRAY);
		Mat grayROI   ; cvtColor(imgROI   ,grayROI   ,CV_BGR2GRAY);
        vector<Point2f> corners;  
		/* find new features set, when current matches are lower then minimum */
		// ..meaning of aquiring new target..
        if (trackedFeatures.size() < min_features * mid_level_percent ) 
			if (FOUND_SOME_MOVEMENT == external_state)
			{
				goodFeaturesToTrack(grayTarget,corners,num_of_maxCornersFeatures,0.01,10);	//  int maxCorners, double qualityLevel, double minDistance,
				cout << "(re-)found " << corners.size() << " features\n";
				// set corners -> trackedFeatures 
				for (int i = 0; i < corners.size(); ++i) {
					trackedFeatures.push_back(corners[i]);
				}

				// if corners.size > minimum*1.3 set GOOD_TRACKING
			}
			else  // loss of features during tracking
			{
				// when near minimum limit - find more feature in ROI around those still alive
				////set LOW_QUALITY_TRACKING

				// if bellow minimum - anounce loss of tracking. and stop..
				cout << "loss of tracking fetures....!"<<endl;
			}

        if(!prevGrayROI.empty()) {
            vector<uchar> status; vector<float> errors;
			// new input is trackedFeatures ->
			// new output is corners
			// status of 1 means correspondence found. 0 otherwise.
            calcOpticalFlowPyrLK(prevGrayROI,grayROI,trackedFeatures,corners,status,errors,Size(10,10));	// corners are 'InOut' array

			/////////////////////
			Mat copy;
			copy	= grayROI.clone();
			int r	= 2;//3
			for( int i = 0; i < corners.size(); i++ )
			{ circle( copy, corners[i], r, 
				Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), 
				-1, 8, 0 );
				//x_cord[i] = corners[i].x;
				//y_cord[i] = corners[i].y;
			}			
			
			imshow("original copy ROI", copy);	

			/////////////////////////////////
			/*  just trial additional code 
			// Holds the colormap version of the image:
			Mat cm_img0;
			// Apply the colormap:
			applyColorMap(copy, cm_img0, COLORMAP_JET);
			// Show the result:
			imshow("cm_img0", cm_img0);*/
			/////////////////////////////////

			/// Create Trackbars
			////cout << TrackbarName << "Alpha x %d" << alphaSlider_max ;

			createTrackbar( TrackbarName, "original copy", 
				&alphaSlider, num_of_maxCornersFeatures, /*on_trackbar*/NULL );
			///*alphaSlider++;
			/*if (alphaSlider>alphaSlider_max) alphaSlider=0;*/
			alphaSlider = countNonZero(status);
			////////////////
			createTrackbar( TrackbarName2, "original copy", 
				&alphaSlider2, alphaSlider_max, /*on_trackbar*/NULL );
			///*alphaSlider++;
			/*if (alphaSlider>alphaSlider_max) alphaSlider=0;*/
			alphaSlider2 = countNonZero(status)/ status.size() * 100.0;
			//////////////////////////////////

            if(alphaSlider * mid_level_percent < status.size()) {
                cout << "cataclysmic error \n";
                rigidTransform	= Mat::eye(3,3,CV_32FC1);	//same as above specific function
                trackedFeatures	.clear();
                prevGrayROI		.release();
                freshStart		= true;
                return;
            } else
                freshStart = false;

            Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,corners,false);
			if (!newRigidTransform.empty())
			{
				float a = newRigidTransform.at<float>(0,0);
				float b = newRigidTransform.at<float>(0,1);
				float c = newRigidTransform.at<float>(1,0);
				float d = newRigidTransform.at<float>(1,1);
			
				float sX ;
				float sign_a = a>=0? 1:(-1);
				sX = sign_a * sqrtf(powf(a,2) + powf(b,2));
				float sY;
				float sign_d = d>=0? 1:(-1);
				sY = sign_d * sqrt(powf(c,2) + powf(d,2));

				// reduce the effect of scale.
				float* tmp = NULL;
				tmp =  newRigidTransform.ptr<float>(0);
				tmp[0] = tmp[0]/sX;
				tmp[1] = tmp[1]/sX;
				tmp =  newRigidTransform.ptr<float>(1);
				tmp[0] = tmp[0]/sY;
				tmp[1] = tmp[1]/sY;
			
				Mat_<float> nrt33 = Mat_<float>::eye(3,3);
				newRigidTransform.copyTo(nrt33.rowRange(0,2));
				rigidTransform *= nrt33;			

				trackedFeatures.clear();
				for (int i = 0; i < status.size(); ++i) {
					if(status[i]) {
						trackedFeatures.push_back(corners[i]);
					}
				}

				TrackPercent = alphaSlider2 ;
			}
			else
			{
				cout << "cataclysmic error 2! \n";
				rigidTransform	= Mat::eye(3,3,CV_32FC1);	//same as above specific function
				trackedFeatures	.clear();
				prevGrayROI		.release();
				freshStart		= true;
				TrackPercent	= 0;
				return;
			}
        }

        for (int i = 0; i < trackedFeatures.size(); ++i) {
            circle(grayROI,trackedFeatures[i],3,Scalar(0,0,255),CV_FILLED);
        }

		grayROI.copyTo(prevGrayROI);
    }
};

////////////////////////////////////////////////////////////////////////////
// next function is used only as demonstration when running as stand-alone.

int stabilizer_main( VideoCapture cap ) {
    
    Mat		frame,orig,orig_warped,tmp;

	Tracker tracker;

    while(cap.isOpened()) {

		////////////////////////////////////////////////
		//			capture new frame
		////////////////////////////////////////////////
		cap >> frame;
        if(frame.empty()) break;

		resize   (frame, frame, Size (240,120) , 0, 0, INTER_CUBIC); 

        frame.copyTo(orig);		
		imshow("orig",orig);

		////////////////////////////////////////////////
		//			make tracking of the 'goodFeatures'

		/* clear points that are out of my desired ROI (center of image) */
		int frame_boundary = 30;  //TODO:make 20 h , 30 w /// sizes are for after resize
								  //CV_EXPORTS_W void rectangle(InputOutputArray img, Point pt1, Point pt2,
								  //	const Scalar& color, int thickness = 1,
								  //	int lineType = LINE_8, int shift = 0);
		Point	TopLeft(frame_boundary, frame_boundary);
		Point	LowRight(orig.size().width - frame_boundary , orig.size().height - frame_boundary);
		Rect	myROI = Rect(TopLeft, LowRight ); 
		
		////////////////////////////////////////////////
		//			make tracking of the 'goodFeatures'
		//			from previous frame to the new one		
        tracker.processImage(orig (myROI),  orig (myROI) , TRACKING_GOOD_QUALITY_TARGET);   // myROI

		////////////////////////////////////////////////
		//	calculate translation from previous to current.
		//	display modified current 
        Mat invTrans = tracker.rigidTransform.inv(DECOMP_SVD);
        warpAffine(orig,orig_warped,invTrans.rowRange(0,2),Size());
        imshow("stabilized",orig_warped);

		////////////////////////////////////////////////
		//	loop termination conditions
        int c=0;
		c = waitKey(10);
			//if (recorded_frames_input)
			//  c = waitKey(0);  // wait for user
        if(c == 27) break;
    }

	cap.release();

	return 0;
}