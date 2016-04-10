#include "calibration.h"

#include "util.h"
#include "intrinsics.h"
#include <iostream>
#include <cstdlib>
using namespace cv;
using namespace std;


Calibration::Calibration()
{
	calibrationEnd = false;
}

void Calibration::readSettings(std::string path)
{
}
void Calibration::saveSettings(std::string path)
{
}


Mat Calibration::createChessboard(int cx, int cy, int blockSize)
{
	int size1 = 6;
	int size2 = 5;
    int height = size1 * blockSize;
    int width = size2 * blockSize;
    // Mat chessBoard(height, width, CV_8UC1, Scalar::all(255));
    // Mat chessBoard(640, 640, CV_8UC1, Scalar::all(255));
    Mat chessBoard(projectorResolutionX, projectorResolutionY, CV_8UC1, Scalar::all(255));
    unsigned char color=255;

     for(int i=0;i<height;i=i+blockSize){
      color=~color;
       for(int j=0;j<width;j=j+blockSize){
       Mat ROI=chessBoard(Rect(cx+i,cy+j,blockSize,blockSize));
       ROI.setTo(Scalar::all(color));
       color=~color;
      }
      if(size1%2==0) color=~color;
     }
     internalPoints.clear();
     for(int i=1;i<size2;i++)
     {
     	for(int j=1;j<size1;j++)
     	{
     		internalPoints.push_back(Point2f(cx+(blockSize*(j)),cy+(blockSize*(i))));
     		// cout<<cx+(blockSize*(j+1))<<" "<<cy+(blockSize*(i+1))<<"\n";
     	}
     }


	return chessBoard;
}

bool Calibration::detectChessboard(Mat image, vector<Point2f> output)
{
	Size patternsize(5,4);
    bool patternfound = findChessboardCorners( image, patternsize, output,
	  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);

    if(patternfound){
    	printf("found\n");
    	return true;
    } else {
    	printf("not found\n");
    	return false;
    }
}

void Calibration::setKinect(libfreenect2::SyncMultiFrameListener *listener)
{
	_listener = listener;
}

pair<int,int> Calibration::projectChessbooard(int blockSize, int x, int y)
{
	if(x<0 && y<0)
	{
		/*srand( time( NULL ) );
		x = rand()%300;
		y = rand()%300;*/
		x = 0;
		y = 0;
	}
	namedWindow("calibration", CV_WINDOW_NORMAL);
	moveWindow("calibration", 0, 0);
	setWindowProperty("calibration", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
// cvNamedWindow("Name", CV_WINDOW_NORMAL);
    // cvSetWindowProperty("Name", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	_chessBoard = createChessboard(x,y,blockSize);
	imshow("calibration", _chessBoard);
	
	return make_pair(x,y);
}
void Calibration::projectFindResult(bool flag, vector<Point2f> points)
{
	if(!flag) projectFindResult(flag);
	else{
		for(int i=0;i<points.size(); i++){
			Mat ROI=_chessBoard(Rect(points[i].x,points[i].y,10,10));
			ROI.setTo(Scalar(150,150,150,150));
		}
		imshow("calibration", _chessBoard);
	}
}

void Calibration::projectFindResult(bool flag)
{
	Mat ROI=_chessBoard(Rect(0,0,100,100));
	if(flag) ROI.setTo(Scalar(0,0,0,0));
	else ROI.setTo(Scalar(150,150,150,150));
	imshow("calibration", _chessBoard);
}
void Calibration::setProjectorResolution(int rx, int ry)
{
	projectorResolutionX = rx;
	projectorResolutionY = ry;
}
void Calibration::collectPoints(libfreenect2::Freenect2Device *dev)
{
	vector<pair<Point2f,Point3f> > result;
	Size patternsize(5,4);
	int blockSize = 30;
	libfreenect2::FrameMap frames;
  	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
  	int numberOfIterations = 5;
  	// pair<int,int> tab[8] = {{200,200},{200,100},{100,100},{100,200},{200,200},{200,100},{100,100},{100,200}};
	pair<int,int> tab[10] = {{200,200},{200,200},{200,200},{200,200},{200,200},{100,100},{100,100},{100,100},{100,100},{100,100}};
	while(!calibrationEnd)
	{
		pair<int,int> projEntryPoint = projectChessbooard(blockSize, 200, 200);
		// pair<int,int> projEntryPoint = projectChessbooard(blockSize, -1, -1);
		// pair<int,int> projEntryPoint = projectChessbooard(blockSize, tab[numberOfIterations-1].first, tab[numberOfIterations-1].second);
        vector<Point2f> chessboardCorners;


		_listener->waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        libfreenect2::Frame bigdepth(1920, 1082, 4);
    	registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth, NULL);
    	Mat depthMap = frameToMat("map", &bigdepth);
        Mat rgbImage =  frameToMat("registered", &registered); // frameToMat("rgb", rgb);
        Mat grayImage;
        cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
        waitKey(3000); // wait before looking for chessboard
        bool patternfound = findChessboardCorners( grayImage, patternsize, chessboardCorners,
                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                           );//+ CALIB_CB_FAST_CHECK);

        if(patternfound){

        	vector<Point3f>		worldCoordinatesChessboard;
			vector<Point2f>		kinectCoordinatesChessboard;
			vector<Point2f>		imageCoordinatesChessboard;
			vector<Point2f>		imageCoordinatesChessboardInternal = internalPoints;
			bool check = true;
			vector<pair<Point2f,Point3f> > tempResult;
			
			int tabSize = 6;
			int tab[10] = {0, 9, 16, 4, 5, 19, 1, 15, 18, 11};
			for(int j = 0; j<tabSize; j++)
			// for(int i=0;i<chessboardCorners.size();i+=7)
			{
				int i = tab[j];
				float x=0,y=0,z=0,color=0;
				registration->getPointXYZRGB(&undistorted, &registered,
					chessboardCorners[i].x,
					chessboardCorners[i].y,
					x,y,z,color);
				if(cvIsNaN(x) || cvIsNaN(y) || cvIsNaN(z))
				{
					check = false;
					break;
				}
				kinectCoordinatesChessboard.push_back(chessboardCorners[i]);
				worldCoordinatesChessboard.push_back(Point3f(x*100,y*100,z*100));
				imageCoordinatesChessboard.push_back(imageCoordinatesChessboardInternal[i]);
			}
			if(check)
			{
				worldCoordinatesChessboardBuffer.push_back(worldCoordinatesChessboard);
				kinectCoordinatesChessboardBuffer.push_back(kinectCoordinatesChessboard);
				imageCoordinatesChessboardBuffer.push_back(imageCoordinatesChessboard);
				printf("PATTERN FOUND\n");
          		projectFindResult(true);
          		projectFindResult(true, imageCoordinatesChessboard);
          		numberOfIterations--;
          		if(numberOfIterations == 0) calibrationEnd = true;
			} else {
				printf("PATTERN NOT FOUND\n");
         	 	projectFindResult(false);
			}

        } else {
          printf("PATTERN NOT FOUND\n");
          projectFindResult(false);
        }

        _listener->release(frames);
		int op = waitKey(1000);
    	cout<<op<<"\n";
    	if(op == 1113997) break; // right enter

	}
	 destroyWindow("calibration");
	 waitKey(100);
	 // return result;
}

void Calibration::calibrate2()
{

	int nPairs = 0;
	for(int i = 0; i < worldCoordinatesChessboardBuffer.size(); i++)
    	nPairs += worldCoordinatesChessboardBuffer[i].size();

	Mat A = Mat::zeros(nPairs*2, 11, CV_64F);
	Mat y = Mat::zeros(nPairs*2, 1, CV_64F);
    
    int counter = 0;
    // for (int i=0; i<nPairs; i++)
    for(int i = 0; i < worldCoordinatesChessboardBuffer.size(); i++) 
    	for(int j = 0; j < worldCoordinatesChessboardBuffer[i].size(); j++) {

    	A.at<double>(2*counter,0) = worldCoordinatesChessboardBuffer[i][j].x;
    	A.at<double>(2*counter,1) = worldCoordinatesChessboardBuffer[i][j].y;
    	A.at<double>(2*counter,2) = worldCoordinatesChessboardBuffer[i][j].z;
    	A.at<double>(2*counter,3) = 1.0;
    	A.at<double>(2*counter,8) = -worldCoordinatesChessboardBuffer[i][j].x * imageCoordinatesChessboardBuffer[i][j].x;
    	A.at<double>(2*counter,9) = -worldCoordinatesChessboardBuffer[i][j].y * imageCoordinatesChessboardBuffer[i][j].x;
    	A.at<double>(2*counter,10) = -worldCoordinatesChessboardBuffer[i][j].z * imageCoordinatesChessboardBuffer[i][j].x;

		A.at<double>(2*counter+1,4) = worldCoordinatesChessboardBuffer[i][j].x;
		A.at<double>(2*counter+1,5) = worldCoordinatesChessboardBuffer[i][j].y;
		A.at<double>(2*counter+1,6) = worldCoordinatesChessboardBuffer[i][j].z;
		A.at<double>(2*counter+1,7) = 1.0;

		A.at<double>(2*counter+1,8) = -worldCoordinatesChessboardBuffer[i][j].x * imageCoordinatesChessboardBuffer[i][j].y;
		A.at<double>(2*counter+1,9) = -worldCoordinatesChessboardBuffer[i][j].y * imageCoordinatesChessboardBuffer[i][j].y;
		A.at<double>(2*counter+1,10) = -worldCoordinatesChessboardBuffer[i][j].z * imageCoordinatesChessboardBuffer[i][j].y;


        y.at<double>(2*counter,0) = imageCoordinatesChessboardBuffer[i][j].x;
        y.at<double>(2*counter+1,0) = imageCoordinatesChessboardBuffer[i][j].y;
    }
    
    Mat result;
	bool r = solve(A, y, result, DECOMP_QR); 
	for (int i=0; i<11; i++) {
        x.push_back(result.at<double>(i, 0));
    }

	cout<<"RESULT "<<result<<"\n";

}

void Calibration::calibrate(){
	cameraMatrix = (Mat1d(3, 3) << 	projectorResolutionX, 0, projectorResolutionX / 2.,
					                0, projectorResolutionY, projectorResolutionY / 2.,
					                0, 0, 1);
	distCoeffs = Mat::zeros(8, 1, CV_64F);

	bool calibrated = false;

	int flags = CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_ZERO_TANGENT_DIST + 
				CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3;
				// CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_FIX_PRINCIPAL_POINT;
	cout << "Calibration START\n";
	vector<vector<Point3f> > vvo(1); //object points
	vector<vector<Point2f> > vvi(1); //image points
	// cout<<"c->worldCoordinatesChessboardBuffer.resize("<<worldCoordinatesChessboardBuffer.size()<<");\n";
	// cout<<"c->imageCoordinatesChessboardBuffer.resize("<<imageCoordinatesChessboardBuffer.size()<<");\n";
	float reprojError;
	for (int i=0; i<worldCoordinatesChessboardBuffer.size(); ++i) {
		for (int j = 0; j<worldCoordinatesChessboardBuffer[i].size(); j++) {
			vvo[0].push_back(worldCoordinatesChessboardBuffer[i][j]);
			vvi[0].push_back(imageCoordinatesChessboardBuffer[i][j]);
			// cout<<"c->worldCoordinatesChessboardBuffer["<<i<<"].push_back(Point3f("<<worldCoordinatesChessboardBuffer[i][j].x<<","
				// <<worldCoordinatesChessboardBuffer[i][j].y<<","<<worldCoordinatesChessboardBuffer[i][j].z<<"));\n";

			// cout<<"c->imageCoordinatesChessboardBuffer["<<i<<"].push_back(Point2f("<<imageCoordinatesChessboardBuffer[i][j].x<<","
				// <<imageCoordinatesChessboardBuffer[i][j].y<<"));\n";
			/*
			if(i>0){
				Mat cameraMatrix1 = (Mat1d(3, 3) << 	projectorResolutionX, 0, projectorResolutionX / 2.,
					                0, projectorResolutionY, projectorResolutionY / 2.,
					                0, 0, 1);
				Mat distCoeffs1 = Mat::zeros(8, 1, CV_64F);
				vector<Mat>	boardRotations1, boardTranslations1;
				reprojError = calibrateCamera(vvo, vvi, cv::Size(projectorResolutionX, projectorResolutionY), cameraMatrix1, distCoeffs1, boardRotations1, boardTranslations1, flags);
				cout<<"//reprojError"<<i<<": "<<reprojError<<"\n";
			}
			//*/

		}
	}

	reprojError = calibrateCamera(vvo, vvi, cv::Size(projectorResolutionX, projectorResolutionY), cameraMatrix, distCoeffs, boardRotations, boardTranslations, flags);
	cout<<"reprojError: "<<reprojError<<"\n";
	calibrated = true;

	Intrinsics intrinsics;
	intrinsics.setup(cameraMatrix, cv::Size(projectorResolutionX, projectorResolutionY));

	int totalPoints = 0;
	double totalErr = 0;	
	vector<float> perViewErrors;	
	perViewErrors.clear();	
	
	perViewErrors.resize(worldCoordinatesChessboardBuffer.size());
	for(int i = 0; i < worldCoordinatesChessboardBuffer.size(); i++) {
		vector<Vec2f> imagePts = project(worldCoordinatesChessboardBuffer[i],i );
		double err = norm(Mat(imagePts), Mat(imageCoordinatesChessboardBuffer[i]), CV_L2);
		int n = worldCoordinatesChessboardBuffer[i].size();
		perViewErrors[i] = sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}
}

vector<Point2f> Calibration::projectPoints2(vector<Point3f> wrldSrc)
{
	vector<Point2f> result;
	for(int i=0;i<wrldSrc.size();i++){
		double a = x[0]*wrldSrc[i].x + x[1]*wrldSrc[i].y + x[2]*wrldSrc[i].z + x[3];
		double b = x[4]*wrldSrc[i].x + x[5]*wrldSrc[i].y + x[6]*wrldSrc[i].z + x[7];
		double c = x[8]*wrldSrc[i].x + x[9]*wrldSrc[i].y + x[10]*wrldSrc[i].z + 1;
		result.push_back(Point2f(a/c, b/c));

	}

    return result;
}

vector<Vec2f> Calibration::project(vector<Point3f> wrldSrc, int i)
 {
	// if (!calibrated)  { vector<ofVec2f> v; v.push_back(ofVec2f(0,0)); return v; }
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(wrldSrc, mr, mt, cameraMatrix, distCoeffs, projected);	
	vector<Vec2f> projectedOF;
	for (int j = 0; j < projected.size(); j++) {
		projectedOF.push_back((projected[j]));
	}
	return projectedOF;
}