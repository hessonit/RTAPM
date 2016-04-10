#include "util.h"


#include <iostream>
using namespace cv;



void showMat(Mat image)
{
	namedWindow( "window", WINDOW_AUTOSIZE );
    imshow("window", image);
    resizeWindow("window", 500,500);
    moveWindow("window", 0, 0);

    waitKey(0);
    destroyWindow("window");
}

Mat frameToMat(std::string type, libfreenect2::Frame *frame)
{
	if(type == "rgb")
	{
		Mat result(1080, 1920, CV_8UC4, frame->data);
		return result;
	}
	else if(type == "ir")
	{
		Mat result(424, 512, CV_32FC1, frame->data);
		return result;
	}
	else if(type == "depth")
	{
		Mat result(424, 512, CV_32FC1, frame->data);
		return result;
	}
	else if(type == "registered")
	{
		Mat result(424, 512, CV_8UC4, frame->data);
		return result;
	}
	else if(type == "map")
	{
		Mat result(1082, 1920, CV_32FC1, frame->data);
		return result;
	}
	return Mat();
}

int doSomething(int s)
{
	Mat image;
    image = imread("./chess1.png", CV_LOAD_IMAGE_GRAYSCALE);
	// image = createChessboard(6,6);
    // namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    // imshow( "Display window", image );                   // Show our image inside it.

    // waitKey(0);                                          // Wait for a keystroke in the window

    std::vector<Point2f> pointBuf;
	Size patternsize(3,3);
    bool patternfound = findChessboardCorners( image, patternsize, pointBuf,
	  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);

    if(patternfound){
    	printf("found\n");
    	// std::cout<<pointBuf;
    	// Mat res(600, 600, CV_8UC1, Scalar(100,100,100));
  		cornerSubPix(image, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		

		drawChessboardCorners(image, patternsize, Mat(pointBuf), patternfound);

		namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    	imshow( "Display window", image );                   // Show our image inside it.
    	waitKey(0);


    } else {
    	printf("not found\n");
    }
    return 0;
}





