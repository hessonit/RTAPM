#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include "pt.h"
#include "calibration.h"

#include <fstream>
#include <cstring> 
#include <cstddef>
#include <cstdint>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <libfreenect2/frame_listener.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "viewer.h"
#include "CTObject.h"
#include "util.h"
#include "Projector.h"


//#include<OGRE\Ogre.h>
//
//int main()
//{
//	std::cout << "HAHAHA";
//	Ogre::Root* root = new Ogre::Root("plugins.cfg", "ogre.cfg", "Ogre.log");
//	delete root;
//	std::getchar();
//	return 0;
//}


int main(int argc, char *argv[])
{
  std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
  std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
  std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;
  bool gpuView = false;

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
  std::cout<<"test(1) or calibration(2) or noCalib(3)\n";
  std::cin >> tt;
}
gpuView = true;


//CALIBRATION CODE (tracking part is around 250 lines lower)
if (tt == 2 || tt == 3){
/////////////////////Initialize Kinect /////////////////////
  int noCalib = 0;
  if(tt == 3) noCalib = 1;


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
  if(noCalib == 0)
    c.collectPoints(dev);
  
  std::cout<<"CALIBRATED\n";
  bool singleUpdate = true;
  double up = 0;
  double right = 0;
  double rotX = 0;
  double rotY = 0;
  double rotZ = 0;
  bool print = true;
  if(noCalib>-1 || c.calibrationEnded())
  {
    if(noCalib == 0)
      c.calibrate();

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    bool endin = false;
    if(!gpuView){
		cv::namedWindow("calibration2", CV_WINDOW_NORMAL);
		cv::moveWindow("calibration2", 0, 0);
      //setWindowProperty("calibration2", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    } else {
      viewer.initialize();
    }
    bool initViewer = true;
    while(!endin)
    {
      std::vector<cv::Point3f> wrldSrc;
	  cv::Mat board(480, 640, CV_8UC4, cv::Scalar::all(255));

      if(!gpuView) cv::imshow("calibration2", board);
      else if(initViewer) {
        initViewer = false;
        libfreenect2::Frame b(640, 480, 4);
        b.data = board.data;
        viewer.addFrame("RGB", &b);
        endin = endin || viewer.render();
      }

      (&listener)->waitForNewFrame(frames);
      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
      registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);
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
            x = static_cast<float>(x+right/((double)c.projectorResolutionX));
            y = static_cast<float>(y+up/((double)c.projectorResolutionY));

            x -= 0.5;
            y -= 0.5;
			double PI = 3.14159265;
            x = static_cast<float>(std::cos(rotX * PI / 180) * x - std::sin(rotX * PI / 180) * y);
            y = static_cast<float>(std::sin(rotX * PI / 180) * x + std::cos(rotX * PI / 180) * y);

            x += 0.5;
            y += 0.5;

            wrldSrc.push_back(cv::Point3f(x*100,
                                      y*100,
                                      z*100));
          }
        }
      }
	  cv::Mat mt, mr, cam, pro;
      if(noCalib == 0)
      { 
          mt = c.boardTranslations[0];
          mr = c.boardRotations[0];
          cam = c.cameraMatrix;
          pro = c.distCoeffs;
          if(print){
            std::cout<<mt<<"\n"<<mr<<"\n"<<cam<<"\n"<<pro<<"\n";
            print = false;
          }
      } else {
		  //std::cout << "DUPA PRINT\n";
        mt = (cv::Mat1f(3,1) << 14.9566527691241,
                            17.7872756163266,
                            -6.798784049003872);

        mr = (cv::Mat1f(3,1) << 2.178677563707194,
                            2.096964130591053,
                            0.1258202253438768);

        cam = (cv::Mat_<double>(3,3) << 1053.314135376467, 0, 670.864138058805,
                                    0, 1059.961617203515, 291.5273582648912,
                                    0, 0, 1);

        pro = (cv::Mat1f(5,1) << 0,0,0,0,0);
        if(singleUpdate){
          singleUpdate = false;
          ////////////////////////////
          right = -40;
          up = 89;
          rotX = -4;
          /////////////////////////////
		}
      }
      std::vector<cv::Point2f> projected;
      std::vector<cv::Point3f> src = (wrldSrc);
      if(wrldSrc.size()>0){
        // projectPoints(src, mr, mt, c.cameraMatrix, c.distCoeffs, projected);  
          projectPoints(src, mr, mt, cam, pro, projected);
		std::vector<cv::Point2f> projectedOF;
        for (int i = 0; i < projected.size(); i++) {
          projectedOF.push_back(projected[i]);
      }
      
        for(int i=0;i<projectedOF.size();i++)
        {
          if(480-projectedOF[i].x >0 && projectedOF[i].y > 0 && 480-projectedOF[i].x < 475 && projectedOF[i].y < 630){

			  cv::Mat ROI=board(cv::Rect(static_cast<int>(projectedOF[i].y), static_cast<int>(480-projectedOF[i].x),2,2));
          ROI.setTo(cv::Scalar(100,100,150,100));
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
        int op = cv::waitKey(50);
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
     if(!gpuView) cv::destroyWindow("calibration2");
     else {
      viewer.stopWindow();
     }
  }

  dev->stop();
  dev->close();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////TRACKING & TESTING //////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

if(tt == 1){

  int test = 0;
  PT ptest;
  std::cout<<"VTKTEST(1) or LIBFREENECT2(2) or PlaneScan(3) or CT Viewer(4) or Reproject(5) or CT data viewer(6)\n";
  std::cin >> test;
  if(test == 1)
    ptest.test1(); 
  if(test == 2)
    ptest.test2(argc, argv);
  if(test == 3)
  {
	  int gpuView = 0;
	  std::cout << "CPU view(0) or GPU view(1)\n";
	  std::cin >> gpuView;
	  cv::Mat mt = (cv::Mat1f(3, 1) << 14.9566527691241,
		  17.7872756163266,
		  -6.798784049003872);

	  cv::Mat mr = (cv::Mat1f(3, 1) << 2.178677563707194,
		  2.096964130591053,
		  0.1258202253438768);

	  cv::Mat cam = (cv::Mat_<double>(3, 3) << 1053.314135376467, 0, 670.864138058805,
		  0, 1059.961617203515, 291.5273582648912,
		  0, 0, 1);

	  cv::Mat pro = (cv::Mat1f(5, 1) << 0, 0, 0, 0, 0);
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
	  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	  dev->setColorFrameListener(&listener);
	  dev->setIrAndDepthFrameListener(&listener);
	  dev->start();
	  Projector *projector = new Projector();
	  projector->setKinect(&listener, dev);
	  projector->setMatrices(mt, mr, cam, pro);
		projector->reprojectPlane(gpuView>0);
	  dev->stop();
	  dev->close();
  }
  if (test == 4) {
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
	  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	  dev->setColorFrameListener(&listener);
	  dev->setIrAndDepthFrameListener(&listener);
	  dev->start();
		  Projector *projector = new Projector();
		  projector->setKinect(&listener, dev);
		  projector->ctProjection("C:\\Users\\Adam\\Desktop\\volumetric data\\bunny-ctscan\\bunny\\", 512, 512, 360);
	  dev->stop();
	  dev->close();
  }
  if (test == 5) {
	  int gpuView = 0;
	  std::cout << "CPU view(0) or GPU view(1)\n";
	  std::cin >> gpuView;
	  cv::Mat mt = (cv::Mat1f(3, 1) << 14.9566527691241,
		  17.7872756163266,
		  -6.798784049003872);

	  cv::Mat mr = (cv::Mat1f(3, 1) << 2.178677563707194,
		  2.096964130591053,
		  0.1258202253438768);

	  cv::Mat cam = (cv::Mat_<double>(3, 3) << 1053.314135376467, 0, 670.864138058805,
		  0, 1059.961617203515, 291.5273582648912,
		  0, 0, 1);

	  cv::Mat pro = (cv::Mat1f(5, 1) << 0, 0, 0, 0, 0);
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
	  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	  dev->setColorFrameListener(&listener);
	  dev->setIrAndDepthFrameListener(&listener);
	  dev->start();
		  Projector *projector = new Projector();
		  projector->setKinect(&listener, dev);
		  projector->setMatrices(mt, mr, cam, pro);
		  projector->reproject(gpuView>0);
	  dev->stop();
	  dev->close();
  }
  if (test == 6)
  {
	  SIDE view = SIDE::BOTTOM;
	  std::string pathBunny = "C:\\Users\\Adam\\Desktop\\volumetric data\\bunny-ctscan\\bunny\\";
	  std::string pathBrain = "C:\\Users\\Adam\\Desktop\\volumetric data\\MRbrain\\MRbrain.";
	  CTObject obj(pathBunny, 512, 512, 360);
	  //CTObject obj(pathBrain, 256, 256, 106);
	  obj.setView(view);
	  obj.readData();
	  //obj.showData();
	  obj.showDataFromView();
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