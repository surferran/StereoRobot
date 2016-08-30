#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = -1;
bool showHist = true;
Point origin ;
Rect selection; 
int vmin = 10, vmax = 256, smin = 30;

static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1;
        break;
    }
}

static void help_camshift()
{
    cout << "\nThis is a demo that shows mean-shift based tracking\n"
            "You select a color objects such as your face and it tracks it.\n"
            "This reads from video camera (0 by default, or the camera number the user enters\n"
            "Usage: \n"
            "   ./camshiftdemo [camera number]\n";

    cout << "\n\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tc - stop the tracking\n"
            "\tb - switch to/from backprojection view\n"
            "\th - show/hide object histogram\n"
            "\tp - pause video\n"
            "To initialize tracking, select the object with mouse\n";
}

const char* keys =
{
    "{@camera_number| 0 | camera number}"
};

int do_camshift_main()
{int argc; const char** argv ;

   trackObject = 0;

    help_camshift();

    VideoCapture cap;
    Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
//    CommandLineParser parser(argc, argv, keys);
    int camNum = 0;//parser.get<int>(0);

    cap.open(camNum);

    if( !cap.isOpened() )
    {
        help_camshift();
        cout << "***Could not initialize capturing...***\n";
        cout << "Current parameter's value: \n";
        ///parser.printMessage();
        return -1;
    }

    namedWindow( "Histogram", 0 );
    namedWindow( "CamShift Demo", 0 );
    setMouseCallback( "CamShift Demo", onMouse, 0 );
    createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
    createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
    createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );

    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false;

    for(;;)
    {
        if( !paused )
        {
            cap >> frame;
            if( frame.empty() )
                break;
        }

        frame.copyTo(image);

        if( !paused )
        {
            cvtColor(image, hsv, COLOR_BGR2HSV);

            if( trackObject )
            {
                int _vmin = vmin, _vmax = vmax;

                inRange(hsv,	Scalar(0  , smin, MIN(_vmin,_vmax)),
								Scalar(180, 256 , MAX(_vmin, _vmax)),	mask);
                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &hue, 1, ch, 1);

                if( trackObject < 0 )
                {
                    Mat roi(hue, selection), maskroi(mask, selection);
                    calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                    normalize(hist, hist, 0, 255, NORM_MINMAX);

                    trackWindow = selection;
                    trackObject = 1;

                    histimg = Scalar::all(0);
                    int binW = histimg.cols / hsize;
                    Mat buf(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf, buf, COLOR_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                        rectangle( histimg, Point(i*binW,histimg.rows),
                                   Point((i+1)*binW,histimg.rows - val),
                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
                    }
                }

                calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                backproj &= mask;
                RotatedRect trackBox = CamShift(backproj, trackWindow,
                                    TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
                if( trackWindow.area() <= 1 )
                {
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                       trackWindow.x + r, trackWindow.y + r) &
                                  Rect(0, 0, cols, rows);
                }

                if( backprojMode )
                    cvtColor( backproj, image, COLOR_GRAY2BGR );
                ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
            }
        }
        else if( trackObject < 0 )
            paused = false;

        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }

        imshow( "CamShift Demo", image );
        imshow( "Histogram", histimg );

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch(c)
        {
        case 'b':
            backprojMode = !backprojMode;
            break;
        case 'c':
            trackObject = 0;
            histimg = Scalar::all(0);
            break;
        case 'h':
            showHist = !showHist;
            if( !showHist )
                destroyWindow( "Histogram" );
            else
                namedWindow( "Histogram", 1 );
            break;
        case 'p':
            paused = !paused;
            break;
        default:
            ;
        }
    }

    return 0;
}

//////////////////////////////////////
int				hsize = 16;
float			hranges[] = {0,180};
const float*	phranges = hranges;
Rect			trackWindow; 
int				frameCounter=0; 

Mat 
	frame,
	hsv,
	hue,
	mask, 
	hist, 
	histimg = Mat::zeros(200, 320, CV_8UC3), 
	backproj;

void reset_camshift_vars()	{

	 backprojMode = false;
	 selectObject = false;
	 trackObject = -1;
	 showHist = true;
	 vmin = 10; vmax = 256; smin = 30;
	showHist		= true;
	histimg			= Scalar::all(0);
	selection		= Rect(151,11,110,210);    ///320 240
	backprojMode	= true;
	selectObject	= false;
	frameCounter	=0;

}
	//trackObject		= -1;//  0-not tracking 
						// -1 first time tracking - need to identify target ROI first. then set value to 1.
						//  1-there is a ready property feature of the target to track. do tracking
int do_camshift_on_Single_current_frame(Mat* frame_in)
{
	
	namedWindow( "Histogram", 0 );
	namedWindow( "CamShift Demo", 0 );
	setMouseCallback( "CamShift Demo", onMouse, 0 );
	createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
	createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
	createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );
	
	showHist		= true;
	selection		= Rect(130,5,95,230);    ///320 240
	backprojMode	= true;
	selectObject	= false;

	//for(;;)
	{
		frame = *frame_in;
		frame.copyTo(image);
		frameCounter++;
		//if( !paused )
		{
			cvtColor(image, hsv, COLOR_BGR2HSV);

			if( trackObject )
			{
				int _vmin = vmin, _vmax = vmax;

				// loop on AutoAdjust for the selected area, for choosing smin,vimn,vmax
				inRange(hsv,	Scalar(0  , smin, MIN(_vmin,_vmax)),
								Scalar(180, 256 , MAX(_vmin, _vmax)),	mask);
				// TODO: test mask result to get most pixels in selsection area then in the whole image

				int ch[] = {0, 0};
				hue.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue, 1, ch, 1);

				if( trackObject < 0 )
				{
					Mat roi(hue, selection), maskroi(mask, selection);
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
					normalize(hist, hist, 0, 255, NORM_MINMAX);

					trackWindow = selection;
					trackObject = 1;

					histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					Mat buf(1, hsize, CV_8UC3);
					for( int i = 0; i < hsize; i++ )
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
					cvtColor(buf, buf, COLOR_HSV2BGR);

					for( int i = 0; i < hsize; i++ )
					{
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
						rectangle( histimg, Point(i*binW,histimg.rows),
							Point((i+1)*binW,histimg.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8 );
					}
				}

				calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				backproj &= mask;
				RotatedRect trackBox = CamShift(backproj, trackWindow,
												TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
				if( trackWindow.area() <= 1 )
				{
					int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
					trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
						trackWindow.x + r, trackWindow.y + r) &
						Rect(0, 0, cols, rows);
				}

				if( backprojMode )
					cvtColor( backproj, image, COLOR_GRAY2BGR );

				/*				trackBox:
					@param center The rectangle mass center.
					@param size Width and height of the rectangle.
					@param angle The rotation angle in a clockwise direction. When the angle is 0, 90, 180, 270 etc.,
					the rectangle becomes an up-right rectangle.
				*/
				ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
				if (frameCounter<30)
					rectangle( image, selection , Scalar(0,0,255), 3 , LINE_AA);
				else
					if (frameCounter<90)
						rectangle( image, trackBox.boundingRect() , Scalar(0,255,0), 3 , LINE_8);
				
			}
		}
		if( selectObject && selection.width > 0 && selection.height > 0 )
		{
			Mat roi(image, selection);
			bitwise_not(roi, roi);
		}

		imshow( "CamShift Demo", image );
		imshow( "Histogram", histimg );

	}

	return 0;
}