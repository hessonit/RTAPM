/** @file main.cpp Main application file. */

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include "pt.h"
#include "calibration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <libfreenect2/frame_listener.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "viewer.h"
#include "VideoFaceDetector.h"



//#define PI 3.14159265
using namespace cv;
/// [main]

 void detectAndDisplay( Mat frame );
  
void fill(Calibration *c)
{
//  c->worldCoordinatesChessboardBuffer.resize(5);
//c->imageCoordinatesChessboardBuffer.resize(5);
//c->worldCoordinatesChessboardBuffer[0].push_back(Point3f(1.47038,25.6647,127.518));
//c->imageCoordinatesChessboardBuffer[0].push_back(Point2f(230,230));
//c->worldCoordinatesChessboardBuffer[0].push_back(Point3f(-2.40759,11.5921,129.929));
//c->imageCoordinatesChessboardBuffer[0].push_back(Point2f(350,260));
//c->worldCoordinatesChessboardBuffer[0].push_back(Point3f(-9.07824,22.0603,128.855));
//c->imageCoordinatesChessboardBuffer[0].push_back(Point2f(260,320));
//c->worldCoordinatesChessboardBuffer[0].push_back(Point3f(1.48915,11.5223,129.147));
//c->imageCoordinatesChessboardBuffer[0].push_back(Point2f(350,230));
//c->worldCoordinatesChessboardBuffer[0].push_back(Point3f(-2.01754,25.7044,127.715));
//c->imageCoordinatesChessboardBuffer[0].push_back(Point2f(230,260));
//c->worldCoordinatesChessboardBuffer[0].push_back(Point3f(-9.56027,11.6547,130.63));
//c->imageCoordinatesChessboardBuffer[0].push_back(Point2f(350,320));
//c->worldCoordinatesChessboardBuffer[1].push_back(Point3f(-5.61583,12.9051,86.4141));
//c->imageCoordinatesChessboardBuffer[1].push_back(Point2f(230,230));
////reprojError1: 33.2399
//c->worldCoordinatesChessboardBuffer[1].push_back(Point3f(-8.147,3.29142,88.2518));
//c->imageCoordinatesChessboardBuffer[1].push_back(Point2f(350,260));
////reprojError1: 1.40785
//c->worldCoordinatesChessboardBuffer[1].push_back(Point3f(-12.7861,10.6147,86.9972));
//c->imageCoordinatesChessboardBuffer[1].push_back(Point2f(260,320));
////reprojError1: 1.23913
//c->worldCoordinatesChessboardBuffer[1].push_back(Point3f(-5.73124,3.28911,88.19));
//c->imageCoordinatesChessboardBuffer[1].push_back(Point2f(350,230));
////reprojError1: 1.22112
//c->worldCoordinatesChessboardBuffer[1].push_back(Point3f(-7.75082,12.9211,86.5215));
//c->imageCoordinatesChessboardBuffer[1].push_back(Point2f(230,260));
////reprojError1: 1.65666
//c->worldCoordinatesChessboardBuffer[1].push_back(Point3f(-12.9733,3.29213,88.2709));
//c->imageCoordinatesChessboardBuffer[1].push_back(Point2f(350,320));
////reprojError1: 1.58698
//c->worldCoordinatesChessboardBuffer[2].push_back(Point3f(9.2857,40.8968,176.794));
//c->imageCoordinatesChessboardBuffer[2].push_back(Point2f(230,230));
////reprojError2: 1.81301
//c->worldCoordinatesChessboardBuffer[2].push_back(Point3f(4.06416,21.5819,180.936));
//c->imageCoordinatesChessboardBuffer[2].push_back(Point2f(350,260));
////reprojError2: 1.91594
//c->worldCoordinatesChessboardBuffer[2].push_back(Point3f(-5.7311,36.3149,178.018));
//c->imageCoordinatesChessboardBuffer[2].push_back(Point2f(260,320));
////reprojError2: 1.98026
//c->worldCoordinatesChessboardBuffer[2].push_back(Point3f(9.02385,21.6181,181.239));
//c->imageCoordinatesChessboardBuffer[2].push_back(Point2f(350,230));
////reprojError2: 1.97105
//c->worldCoordinatesChessboardBuffer[2].push_back(Point3f(4.45667,40.9188,176.89));
//c->imageCoordinatesChessboardBuffer[2].push_back(Point2f(230,260));
////reprojError2: 1.92418
//c->worldCoordinatesChessboardBuffer[2].push_back(Point3f(-6.33989,21.6516,181.52));
//c->imageCoordinatesChessboardBuffer[2].push_back(Point2f(350,320));
////reprojError2: 2.09446
//c->worldCoordinatesChessboardBuffer[3].push_back(Point3f(-2.0554,20.8091,110.923));
//c->imageCoordinatesChessboardBuffer[3].push_back(Point2f(230,230));
////reprojError3: 2.58745
//c->worldCoordinatesChessboardBuffer[3].push_back(Point3f(-4.88353,8.24644,113.241));
//c->imageCoordinatesChessboardBuffer[3].push_back(Point2f(350,260));
////reprojError3: 2.53914
//c->worldCoordinatesChessboardBuffer[3].push_back(Point3f(-10.9804,17.6909,112.296));
//c->imageCoordinatesChessboardBuffer[3].push_back(Point2f(260,320));
////reprojError3: 2.49171
//c->worldCoordinatesChessboardBuffer[3].push_back(Point3f(-2.09185,7.91236,112.89));
//c->imageCoordinatesChessboardBuffer[3].push_back(Point2f(350,230));
////reprojError3: 2.59167
//c->worldCoordinatesChessboardBuffer[3].push_back(Point3f(-4.8123,20.6291,111.589));
//c->imageCoordinatesChessboardBuffer[3].push_back(Point2f(230,260));
////reprojError3: 2.60647
//c->worldCoordinatesChessboardBuffer[3].push_back(Point3f(-11.0977,8.26499,113.496));
//c->imageCoordinatesChessboardBuffer[3].push_back(Point2f(350,320));
////reprojError3: 2.55827
//c->worldCoordinatesChessboardBuffer[4].push_back(Point3f(5.26052,35.5803,157.534));
//c->imageCoordinatesChessboardBuffer[4].push_back(Point2f(230,230));
////reprojError4: 2.75705
//c->worldCoordinatesChessboardBuffer[4].push_back(Point3f(0.974748,17.413,160.713));
//c->imageCoordinatesChessboardBuffer[4].push_back(Point2f(350,260));
////reprojError4: 2.75382
//c->worldCoordinatesChessboardBuffer[4].push_back(Point3f(-7.75814,31.2617,159.663));
//c->imageCoordinatesChessboardBuffer[4].push_back(Point2f(260,320));
////reprojError4: 2.83346
//c->worldCoordinatesChessboardBuffer[4].push_back(Point3f(5.33967,17.3253,159.904));
//c->imageCoordinatesChessboardBuffer[4].push_back(Point2f(350,230));
////reprojError4: 2.85609
//c->worldCoordinatesChessboardBuffer[4].push_back(Point3f(1.81956,35.6408,157.801));
//c->imageCoordinatesChessboardBuffer[4].push_back(Point2f(230,260));
////reprojError4: 2.85811
//c->worldCoordinatesChessboardBuffer[4].push_back(Point3f(-7.85125,16.6237,161.579));
//c->imageCoordinatesChessboardBuffer[4].push_back(Point2f(350,320));
////reprojError4: 3.06559


}

void fill2(Calibration *c)
{
  
  c->x.push_back(-3.05079065266801e-06);
  c->x.push_back(1.745764808531191e-06);

  c->x.push_back(1.533876324860234e-05);
  c->x.push_back(1.769400497144561e-07);

  c->x.push_back(-3.700371994818713e-06);
  c->x.push_back(2.117477055129764e-06);

  c->x.push_back(1.860472789591281e-05);
  c->x.push_back(2.146145308766468e-07);

  c->x.push_back(0.002179859437507333);
  c->x.push_back(-0.001247388738361842);

  c->x.push_back(-0.01095989609301709);
}

#include "util.h"
#include "opencv2/opencv.hpp"
int main(int argc, char *argv[])
/// [main]
{
  std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
  std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
  std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;
  // std::cout << doSomething(5) <<"\n";
  // createChessboard(6,4);
  // doSomething(5);
  // int a,b;
  // std::cin>>a>>b;
  // Calibration c;
  // showMat(c.createChessboard(a,b,20));
  // c.createChessboard(6,4,50);
  bool gpuView = false;

  // showMat(frameToMat("rgb", NULL));

SimpleViewer viewer; 
viewer.setSize(480, 640);
int tt = -1;
// ./Master 2 0 
// 2 -> kalibracja 0 -> projekcja gpu 
if(argc>1)
  {
    int x = atoi(argv[1]);
    tt = x;
    if(argc > 2)
    {
      x = atoi(argv[2]);
      if(x == 0) gpuView = true;
    }
  }
else {
  std::cout<<"test(1) or calibration(2) or noCalib(3) or calibPoint(4) or noCalib2(5) or calib2(6)\n";
  std::cin >> tt;
}
gpuView = true;
if (tt == 2 || tt == 3 || tt == 4 || tt == 5 || tt == 6){
/////////////////////Initialize Kinect /////////////////////
  int noCalib = 0;
  if(tt == 3) noCalib = 1;
  if(tt == 4) noCalib = 2;
  if(tt == 5) noCalib = 3;
  if(tt == 6) noCalib = 4;

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
  libfreenect2::setGlobalLogger(NULL);

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return 0;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
  dev = freenect2.openDevice(serial, pipeline);
  // signal(SIGINT,sigint_handler);
  bool protonect_shutdown = false;

/// [listeners]
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  
  dev->start();
  std::vector<double> q;
  
//*

  Calibration c;
  c.setKinect(&listener);
  c.setProjectorResolution(480, 640); 
  if(noCalib == 0 || noCalib == 4)
    c.collectPoints(dev);
  
  std::cout<<"CALIBRATED\n";
  bool singleUpdate = true;
  double up = 0;
  double right = 0;
  double rotX = 0;
  double rotY = 0;
  double rotZ = 0;
  bool print = true;
  if(noCalib>-1 || c.calibrationEnd)
  {
    if(noCalib == 0)
      c.calibrate();
    if(noCalib == 4)
      c.calibrate2();
    if(noCalib == 2){
      fill(&c);
      c.calibrate();
    }
    if(noCalib == 3){
      fill2(&c);
    }

    // std::cout<<"trans"<<c.boardTranslations[0]<<"\n";
    // std::cout<<"rot"<<c.boardRotations[0]<<"\n"; 
    // std::cout<<c.cameraMatrix<<"\n"<<c.distCoeffs<<"\n";

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    bool endin = false;
    // namedWindow( "calibration2", WINDOW_AUTOSIZE );
    if(!gpuView){
      namedWindow("calibration2", CV_WINDOW_NORMAL);
      moveWindow("calibration2", 0, 0);
      setWindowProperty("calibration2", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    } else {
      viewer.initialize();
    }
    // Mat board(600, 600, CV_8UC4, Scalar::all(255));
    // imshow("calibration", board);
    bool initViewer = true;
    while(!endin)
    {
      std::vector<Point3f> wrldSrc;
      Mat board(480, 640, CV_8UC4, Scalar::all(255)); 

      if(!gpuView) imshow("calibration2", board);
      else if(initViewer) {
        initViewer = false;
        libfreenect2::Frame b(640, 480, 4);
        b.data = board.data;
        viewer.addFrame("RGB", &b);
        endin = endin || viewer.render();
      }
      // std::cout<<board.size();
      // waitKey(1000);
      (&listener)->waitForNewFrame(frames);
      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
      libfreenect2::Frame bigdepth(1920, 1082, 4);
      // bigdepth->data = new float[1920*1080]
      // int *lol = new int[512*424];
      registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth, NULL);
      // std::cout<<"REGISTERED\n";
      // registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth, lol);
      // Mat depthMap = frameToMat("map", &bigdepth);
      // delete[] lol;
      for(int i=0;i<512;i++)
      {
        for(int j=0;j<424;j++)
        {
          float x=0,y=0,z=0,color=0;
          registration->getPointXYZRGB(&undistorted, &registered,
                                        i, j,
                                        x, y, z, color);

          if(z>0.5 && z<1.7)
          {
            if(right > 0)
              std::cout << right/((double)c.projectorResolutionX);
            x = x+right/((double)c.projectorResolutionX);
            y = y+up/((double)c.projectorResolutionY);

            x -= 0.5;
            y -= 0.5;
			double PI = 3.14159265;
            x = std::cos(rotX * PI / 180) * x - std::sin(rotX * PI / 180) * y;
            y = std::sin(rotX * PI / 180) * x + std::cos(rotX * PI / 180) * y;

            x += 0.5;
            y += 0.5;

            wrldSrc.push_back(Point3f(x*100,
                                      y*100,
                                      z*100));
          }
        }
      }
      // std::cout<<"RIGHT: "<<right<<" "<<up<<"\n";
      Mat mt, mr, cam, pro;
      if(noCalib == 0 || noCalib >= 2)
      {
        if(noCalib == 0 || noCalib == 2){
          mt = c.boardTranslations[0];
          mr = c.boardRotations[0];
          cam = c.cameraMatrix;
          pro = c.distCoeffs;
          if(print){
            std::cout<<mt<<"\n"<<mr<<"\n"<<cam<<"\n"<<pro<<"\n";
            print = false;
          }
        }


      } else {
/*
[11.65937905975672;
  18.99402011826748;
  6.990270715933229]
[-2.368337012787725;
  -2.295412516802321;
  0.01994845139153921]
[925.0583139800787, 0, 627.6128831042244;
  0, 985.6725689662501, 264.3100286155804;
  0, 0, 1]
[0;
  0;
  0;
  0;
  0]
// reprojection error = 2.66005
*/
        mt = (Mat1f(3,1) << 14.9566527691241,
                            17.7872756163266,
                            -6.798784049003872);

        mr = (Mat1f(3,1) << 2.178677563707194,
                            2.096964130591053,
                            0.1258202253438768);

        cam = (Mat_<double>(3,3) << 1053.314135376467, 0, 670.864138058805,
                                    0, 1059.961617203515, 291.5273582648912,
                                    0, 0, 1);

        pro = (Mat1f(5,1) << 0,0,0,0,0);
        if(singleUpdate){
          singleUpdate = false;
          ////////////////////////////
          right = -40;
          up = 89;
          rotX = -4;
          /////////////////////////////
      }
        // std::cout<<"RIGHT: "<<right<<" UP: "<<up<<"\n";
      }
      std::vector<Point2f> projected;    
      std::vector<Point3f> src = (wrldSrc);    
      if(wrldSrc.size()>0){
        // projectPoints(src, mr, mt, c.cameraMatrix, c.distCoeffs, projected);  
        if(noCalib <= 2)
          projectPoints(src, mr, mt, cam, pro, projected);
        if(noCalib > 2)
          projected = c.projectPoints2(src);
		std::vector<Point2f> projectedOF;
        for (int i = 0; i < projected.size(); i++) {
          projectedOF.push_back(projected[i]);
      }
      
        for(int i=0;i<projectedOF.size();i++)
        {
          if(480-projectedOF[i].x >0 && projectedOF[i].y > 0 && 480-projectedOF[i].x < 475 && projectedOF[i].y < 630){
          // std::cout<<projectedOF[i].y<<" "<<480-projectedOF[i].x<<"\n";

          Mat ROI=board(Rect(projectedOF[i].y, 480-projectedOF[i].x,2,2)); 
          ROI.setTo(Scalar(100,100,150,100));
        }
        }
        if(!gpuView) imshow("calibration2", board);
        else {
          libfreenect2::Frame b(640, 480, 4);
          b.data = board.data;
          viewer.addFrame("RGB", &b);
          endin = endin || viewer.render();
        }
      }
      (&listener)->release(frames);
      
      if(!gpuView){
        int op = waitKey(50);
        // if(op != -1)
          // std::cout<<"OP: "<<(char)(op)<<" "<<op<<"\n";
        int con = 5;
        if(op == 100 || (char)(op)=='d') right -=1;
        if(op == 115 || (char)(op)=='s') up +=1;
        if(op == 97 || (char)(op)=='a') right +=1;
        if(op == 119 || (char)(op)=='w') up -=1;

        if(op == 114 || (char)(op)=='r') rotX -= 0.5;
        if(op == 102 || (char)(op)=='f') rotX += 0.5;
        if(op == 116) rotY += 0.5;
        if(op == 103) rotY -= 0.5;
        if(op == 121) rotZ -= 0.5;
        if(op == 104) rotZ += 0.5;

        if(op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
        {
         std::cout<<"right = "<< right<<";\nup = "<<up<<";\nrotX = "<<rotX<<";\n";
         break;
        }
      } else {
        right = viewer.offsetX;
        up = viewer.offsetY;
        rotX = viewer.rot;
      }

    }
     if(!gpuView) destroyWindow("calibration2");
     else {
      viewer.stopWindow();
     }
  }







  dev->stop();
  dev->close();
}
if(tt == 1){


// Calibration c;
// c.setProjectorResolution(480, 640);
// c.projectChessbooard(25,100,100);
// int op = waitKey(5000);
// std::cout<<op<<"\n";
// if(op == 10) std::cout<<"en\n";

  int test = 0;
  PT ptest;
  std::cout<<"Test1(1) or Test2(2) or FaceTrack(3) or FaceTrackFast(4)\n";
  std::cin >> test;
  if(test == 1)
    ptest.test1(); 
  if(test == 2)
    ptest.test2(argc, argv);
  if(test == 3)
  {
    // INIT KINECT
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::setGlobalLogger(NULL);
    if(freenect2.enumerateDevices() == 0){
      std::cout << "no device connected!" << std::endl;
      return 0;
    }
    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    if(!pipeline)
          pipeline = new libfreenect2::OpenGLPacketPipeline();
    dev = freenect2.openDevice(serial, pipeline);
    // signal(SIGINT,sigint_handler);
    bool shutdown = false;
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);


    libfreenect2::Frame bigdepth(1920, 1082, 4); // for registration ?

    /////////////////////////////////////////////////////////////////////////////////////
      String face_cascade_name = "C:\\data\\haarcascades\\haarcascade_frontalface_alt.xml";
      String eyes_cascade_name = "C:\\data\\haarcascades\\haarcascade_eye_tree_eyeglasses.xml";
      CascadeClassifier face_cascade;
      CascadeClassifier eyes_cascade;
	  std::string window_name = "FaceTrack";
      RNG rng(12345);

      if( !face_cascade.load( face_cascade_name ) )
        { 
          printf("---(!)Error loading\n"); 
          return -1; 
        }
      if( !eyes_cascade.load( eyes_cascade_name ) )
        {
          printf("--(!)Error loading\n"); 
          return -1; 
        }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    namedWindow("FaceTrack", CV_WINDOW_NORMAL);
    moveWindow("FaceTrack", 0, 0);
    setWindowProperty("FaceTrack", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    bool tracking = false;
    Rect roi_b;
    int maxCorners = 100;
	std::vector<Point2f> cornersA;
    cornersA.reserve(maxCorners);
    while(!shutdown)
    {

      (&listener)->waitForNewFrame(frames);
      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
      
      registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

      ////////////////////FACE TRACKING HERE////////////////////////////
      Mat frame = frameToMat("registered", &registered);
      Mat old_frame;
      if(!tracking){
        std::vector<Rect> faces;
        Mat frame_gray;

        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        equalizeHist( frame_gray, frame_gray );
        old_frame = frame_gray;
        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) ); // minsize
        /*
        for( size_t i = 0; i < faces.size(); i++ )
        {
          Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
          ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
          Mat faceROI = frame_gray( faces[i] );
          std::vector<Rect> eyes;
          eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(15, 15) );
          for( size_t j = 0; j < eyes.size(); j++ )
           {
             Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
             int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
             circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
           }
        }
        imshow( "FaceTrack", frame );
        /*/
        // Rect roi_b;
        Rect roi_c;

        size_t ic = 0; // ic is index of current element
        int ac = 0; // ac is area of current element
        size_t ib = 0; // ib is index of biggest element
        int ab = 0; // ab is area of biggest element
        for (ic = 0; ic < faces.size(); ic++) // Iterate through all current elements (detected faces)
        {
            roi_c.x = faces[ic].x;
            roi_c.y = faces[ic].y;
            roi_c.width = (faces[ic].width);
            roi_c.height = (faces[ic].height);
            ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
            ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element
            if (ac > ab)
            {
                ib = ic;
                roi_b.x = faces[ib].x;
                roi_b.y = faces[ib].y;
                roi_b.width = (faces[ib].width);
                roi_b.height = (faces[ib].height);
            }

            Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window - live stream from camera
            Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
            rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
        }
        // if(faces.size() > 0) tracking = true;
        // imshow( "FaceTrack", frame );
        //*/

          Mat imgA = frame_gray(roi_b);
          int win_size = 15;
          // int maxCorners = 100; 
          double qualityLevel = 0.05; 
          double minDistance = 2.0; 
          int blockSize = 3; 
          double k = 0.04; 
          // vector<Point2f> cornersA; 
          // cornersA.reserve(maxCorners); 
          goodFeaturesToTrack( imgA,cornersA,maxCorners,qualityLevel,minDistance,Mat(),blockSize,true);
          cornerSubPix( imgA, cornersA, Size( win_size, win_size ), Size( -1, -1 ), 
                        TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

          // show good Features to track
           for(int i=0;i<cornersA.size();i++){
              Mat ROI;
              ROI=frame(Rect(cornersA[i].x+roi_b.x, cornersA[i].y+roi_b.y,2,2)); 
              ROI.setTo(Scalar(10,10,150,100));
            }
		    
          for(int i=0;i<cornersA.size();i++){
              cornersA[i].x+=roi_b.x;
              cornersA[i].y+=roi_b.y;
            }
        // std::cout<<"F"<<frame.size();
        imshow( "FaceTrack", frame );
        // int op = waitKey(100);
        // imshow( "FaceTrack", frame );
      } else {

        //optic flow tracking
		std::vector<Point2f> cornersB;
        cornersB.reserve(maxCorners);
        std::vector<uchar> features_found; 
        // features_found.reserve(maxCorners);
        // std::vector<float> feature_errors; 
        // feature_errors.reserve(maxCorners);
        Mat feature_errors;
        int win_size = 15;
        Mat frame_gray;
        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        // equalizeHist( frame_gray, frame_gray );

        std::cout<<"OFLOW STOART\n";
        calcOpticalFlowPyrLK( old_frame, old_frame, cornersA, cornersB, features_found, feature_errors,
            Size( 20, 20 ), 1
            ,cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.3 ), 0 );
        std::cout<<"OFLOW END\n";
        for( int i=0; i < features_found.size(); i++ ){
        Point p0( ceil( cornersA[i].x ), ceil( cornersA[i].y ) );
        Point p1( ceil( cornersB[i].x ), ceil( cornersB[i].y ) );
        line( frame, p0, p1, CV_RGB(255,255,255), 2 );
        } 
        imshow( "FaceTrack", frame );
      }
      ////////////////////////////////////////////////////////////////////

    int op = waitKey(1);
        // if(op != -1)
          // std::cout<<"OP: "<<(char)(op)<<" "<<op<<"\n";
        
        if(op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
        {
         shutdown = true;
         destroyWindow("FaceTrack");
        }
 
      /////////////////////////////////////////////////
      listener.release(frames);
    }
    dev->stop();
    dev->close();
  }
  if (test == 4) {
	  // INIT KINECT
	  libfreenect2::Freenect2 freenect2;
	  libfreenect2::Freenect2Device *dev = 0;
	  libfreenect2::PacketPipeline *pipeline = 0;
	  libfreenect2::setGlobalLogger(NULL);
	  if (freenect2.enumerateDevices() == 0) {
		  std::cout << "no device connected!" << std::endl;
		  return 0;
	  }
	  std::string serial = freenect2.getDefaultDeviceSerialNumber();
	  if (!pipeline)
		  pipeline = new libfreenect2::OpenGLPacketPipeline();
	  dev = freenect2.openDevice(serial, pipeline);
	  // signal(SIGINT,sigint_handler);
	  bool shutdown = false;
	  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	  libfreenect2::FrameMap frames;
	  dev->setColorFrameListener(&listener);
	  dev->setIrAndDepthFrameListener(&listener);
	  dev->start();

	  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);


	  libfreenect2::Frame bigdepth(1920, 1082, 4); // for registration ?

	  namedWindow("FaceTrack", CV_WINDOW_NORMAL);
	  moveWindow("FaceTrack", 0, 0);
	  setWindowProperty("FaceTrack", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	  bool tracking = false;
	  Rect roi_b;
	  int maxCorners = 100;
	  std::vector<Point2f> cornersA;
	  cornersA.reserve(maxCorners);
	  const cv::String    CASCADE_FILE("C:\\data\\haarcascades\\haarcascade_frontalface_alt.xml");

	  VideoFaceDetector detector(CASCADE_FILE);
	  while (!shutdown)
	  {

		  (&listener)->waitForNewFrame(frames);
		  libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		  libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		  registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		  Mat frame = frameToMat("registered", &registered);

		  detector.getFrameAndDetect(frame);

		  rectangle(frame, detector.face(), cv::Scalar(255, 0, 0));
		  circle(frame, detector.facePosition(), 30, cv::Scalar(0, 255, 0));

		  imshow("FaceTrack", frame);


		  int op = waitKey(1);
		  if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
		  {
			  shutdown = true;
			  destroyWindow("FaceTrack");
		  }
		  listener.release(frames);
	  }
	  dev->stop();
	  dev->close();
  }
  if (test == 5) {
	  // INIT KINECT
	  libfreenect2::Freenect2 freenect2;
	  libfreenect2::Freenect2Device *dev = 0;
	  libfreenect2::PacketPipeline *pipeline = 0;
	  libfreenect2::setGlobalLogger(NULL);
	  if (freenect2.enumerateDevices() == 0) {
		  std::cout << "no device connected!" << std::endl;
		  return 0;
	  }
	  std::string serial = freenect2.getDefaultDeviceSerialNumber();
	  if (!pipeline)
		  pipeline = new libfreenect2::OpenGLPacketPipeline();
	  dev = freenect2.openDevice(serial, pipeline);
	  // signal(SIGINT,sigint_handler);
	  bool shutdown = false;
	  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	  libfreenect2::FrameMap frames;
	  dev->setColorFrameListener(&listener);
	  dev->setIrAndDepthFrameListener(&listener);
	  dev->start();

	  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);


	  libfreenect2::Frame bigdepth(1920, 1082, 4); // for registration ?

	  namedWindow("FaceTrack", CV_WINDOW_NORMAL);
	  moveWindow("FaceTrack", 0, 0);
	  setWindowProperty("FaceTrack", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	  bool tracking = false;
	  Rect roi_b;
	  int maxCorners = 100;
	  std::vector<Point2f> cornersA;
	  cornersA.reserve(maxCorners);
	 
	  while (!shutdown)
	  {

		  (&listener)->waitForNewFrame(frames);
		  libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		  libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		  registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		  Mat frame = frameToMat("registered", &registered);

		  ///////////////////////////STUFF BEG///////////////////////////////////////////////////////////



		  ///////////////////////////STUFF END///////////////////////////////////////////////////////////

		  imshow("FaceTrack", frame);


		  int op = waitKey(1);
		  if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
		  {
			  shutdown = true;
			  destroyWindow("FaceTrack");
		  }
		  listener.release(frames);
	  }
	  dev->stop();
	  dev->close();
  }


}

  return 0;
}


//if (test == 5) {
//	// INIT KINECT
//	libfreenect2::Freenect2 freenect2;
//	libfreenect2::Freenect2Device *dev = 0;
//	libfreenect2::PacketPipeline *pipeline = 0;
//	libfreenect2::setGlobalLogger(NULL);
//	if (freenect2.enumerateDevices() == 0) {
//		std::cout << "no device connected!" << std::endl;
//		return 0;
//	}
//	std::string serial = freenect2.getDefaultDeviceSerialNumber();
//	if (!pipeline)
//		pipeline = new libfreenect2::OpenGLPacketPipeline();
//	dev = freenect2.openDevice(serial, pipeline);
//	// signal(SIGINT,sigint_handler);
//	bool shutdown = false;
//	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
//	libfreenect2::FrameMap frames;
//	dev->setColorFrameListener(&listener);
//	dev->setIrAndDepthFrameListener(&listener);
//	dev->start();
//
//	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
//	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
//
//
//	libfreenect2::Frame bigdepth(1920, 1082, 4); // for registration ?
//
//	namedWindow("FaceTrack", CV_WINDOW_NORMAL);
//	moveWindow("FaceTrack", 0, 0);
//	setWindowProperty("FaceTrack", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
//
//	bool tracking = false;
//	Rect roi_b;
//	int maxCorners = 100;
//	std::vector<Point2f> cornersA;
//	cornersA.reserve(maxCorners);
//
//	while (!shutdown)
//	{
//
//		(&listener)->waitForNewFrame(frames);
//		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
//		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
//
//		registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);
//
//		Mat frame = frameToMat("registered", &registered);
//
//		///////////////////////////STUFF BEG///////////////////////////////////////////////////////////
//
//
//
//		///////////////////////////STUFF END///////////////////////////////////////////////////////////
//
//		imshow("FaceTrack", frame);
//
//
//		int op = waitKey(1);
//		if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
//		{
//			shutdown = true;
//			destroyWindow("FaceTrack");
//		}
//		listener.release(frames);
//	}
//	dev->stop();
//	dev->close();
//}
//}