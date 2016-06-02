#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>


#include <vtkSphere.h>
#include <vtkSampleFunction.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkColorTransferFunction.h>
#include <vtkPiecewiseFunction.h>
#include <vtkVolumeProperty.h>
#include <vtkCamera.h>
#include <vtkImageShiftScale.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkXMLImageDataReader.h>


// Test1
#include "pt.h"
//#include "k2g.h"
//#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

// extra headers for writing out ply file
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/time.h>
//#include <pcl/io/ply_io.h>


// Test2
#include <iostream>
#include <signal.h>


#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
/// [headers]
// #include <libfreenect2/libfreenect2.hpp>
// #include <libfreenect2/frame_listener_impl.h>
// #include <libfreenect2/registration.h>
// #include <libfreenect2/packet_pipeline.h>
// #include <libfreenect2/logger.h>
/// [headers]
// #ifdef EXAMPLES_WITH_OPENGL_SUPPORT
#include "viewer.h"
#include "util.h"


// #include <fstream>
// #include <cstdlib>
using namespace cv;

//struct PlySaver{
//
//  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera): 
//           cloud_(cloud), binary_(binary), use_camera_(use_camera){}
//
//  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
//  bool binary_;
//  bool use_camera_;
//
//};


//void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
//{
//  std::string pressed;
//  pressed = event.getKeySym();
//  PlySaver * s = (PlySaver*)data;
//  if(event.keyDown ())
//  {
//    if(pressed == "s")
//    {
//      
//      pcl::PLYWriter writer;
//      std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
//      std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
//      writer.write ("cloud_" + now, *(s->cloud_), s->binary_, s->use_camera_);
//      
//      std::cout << "saved " << "cloud_" + now + ".ply" << std::endl;
//    }
//  }
//}

PT::PT()
{
  // protonect_shutdown = false;
}
bool protonect_shutdown = false;
// Test kinect (Test2)
// void sigint_handler(int s)
// {
//   protonect_shutdown = true;
// }


void PT::test2(int argc, char *argv[])
{
//  std::string program_path(argv[0]);
//  std::cerr << "Environment variables: LOGFILE=<protonect.log>" << std::endl;
//  std::cerr << "Usage: " << program_path << " [gl | cl | cpu] [<device serial>] [-noviewer]" << std::endl;
//  size_t executable_name_idx = program_path.rfind("Master");
//
//  std::string binpath = "/";
//
//  if(executable_name_idx != std::string::npos)
//  {
//    binpath = program_path.substr(0, executable_name_idx);
//  }
//
//#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
//  // avoid flooing the very slow Windows console with debug messages
//  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
//#else
//  // create a console logger with debug level (default is console logger with info level)
///// [logging]
//  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
///// [logging]
//#endif
///// [file logging]
//  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
//  if (filelogger->good())
//    libfreenect2::setGlobalLogger(filelogger);
///// [file logging]
//
///// [context]
//  libfreenect2::Freenect2 freenect2;
//  libfreenect2::Freenect2Device *dev = 0;
//  libfreenect2::PacketPipeline *pipeline = 0;
///// [context]
//
///// [discovery]
//  if(freenect2.enumerateDevices() == 0)
//  {
//    std::cout << "no device connected!" << std::endl;
//    return;
//  }
//
//  std::string serial = freenect2.getDefaultDeviceSerialNumber();
///// [discovery]
//
//  bool viewer_enabled = true;
//
//  for(int argI = 1; argI < argc; ++argI)
//  {
//    const std::string arg(argv[argI]);
//
//    if(arg == "cpu")
//    {
//      if(!pipeline)
///// [pipeline]
//        pipeline = new libfreenect2::CpuPacketPipeline();
///// [pipeline]
//    }
//    else if(arg == "gl")
//    {
//#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
//      if(!pipeline)
//        pipeline = new libfreenect2::OpenGLPacketPipeline();
//#else
//      std::cout << "OpenGL pipeline is not supported!" << std::endl;
//#endif
//    }
//    else if(arg == "cl")
//    {
//#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
//      if(!pipeline)
//        pipeline = new libfreenect2::OpenCLPacketPipeline();
//#else
//      std::cout << "OpenCL pipeline is not supported!" << std::endl;
//#endif
//    }
//    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
//    {
//      serial = arg;
//    }
//    else if(arg == "-noviewer")
//    {
//      viewer_enabled = false;
//    }
//    else
//    {
//      std::cout << "Unknown argument: " << arg << std::endl;
//    }
//  }
//
//  if(pipeline)
//  {
///// [open]
//    dev = freenect2.openDevice(serial, pipeline);
///// [open]
//  }
//  else
//  {
//    dev = freenect2.openDevice(serial);
//  }
//
//  if(dev == 0)
//  {
//    std::cout << "failure opening device!" << std::endl;
//    return;
//  }
//
//  //signal(SIGINT,sigint_handler);
//  protonect_shutdown = false;
//
///// [listeners]
//  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
//  libfreenect2::FrameMap frames;
//
//  dev->setColorFrameListener(&listener);
//  dev->setIrAndDepthFrameListener(&listener);
///// [listeners]
//
///// [start]
//  dev->start();
//
//  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
//  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
///// [start]
//
///// [registration setup]
//  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
//  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
///// [registration setup]
//
//  size_t framecount = 0;
////#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
//  // viewer_enabled = false;
//  // SimpleViewer viewer; 
//  SimpleViewer viewer;
//  if (viewer_enabled)
//    viewer.initialize();
////#else
//  //viewer_enabled = false;
////#endif
//
//bool guard = true;
///// [loop start]
//  while(!protonect_shutdown)
//  {
//    listener.waitForNewFrame(frames);
//    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
//    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
//    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
///// [loop start]
//
///// [registration]
//    registration->apply(rgb, depth, &undistorted, &registered);
///// [registration]
//
//    if(guard)
//    {
//      // Mat temp = frameToMat("depth", depth);
//      // showMat(temp);
//      // temp = frameToMat("registered", &undistorted);
//      // showMat(temp);
//      // temp = frameToMat("registered", &registered);
//      // showMat(temp);
//      // std::cout<<temp<<"\n";
//    //   vector<Point2f> output;
//    //   Size patternsize(5,4);
//    //   Mat image2 = frameToMat("rgb", rgb);
//    //   Mat image;
//    //   cvtColor(image2, image, CV_BGR2GRAY);
//    //   printf("/////////////////////////////////////////////////////////////////////LOOKING?????????????????????\n");
//    //   bool patternfound = findChessboardCorners( image, patternsize, output,
//    // CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
//    //     + CALIB_CB_FAST_CHECK);
//
//    //   if(patternfound){
//    //   printf("/////////////////////////////////////////////////////////////////////found\n");
//    //   // return true;
//    //   cornerSubPix(image, output, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//    
//
//    //   drawChessboardCorners(image, patternsize, Mat(output), patternfound);
//
//    //   showMat(image);
//
//
//    // } else {
//    //   printf("///////////////////////////////////////////////////////////////NOT found\n");
//    //   // showMat(image);
//    //   // return false;
//    // }
//
//
//      guard = false;
//    }
//
//    
//
//    framecount++;
//    if (!viewer_enabled)
//    {
//      if (framecount % 100 == 0)
//        std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
//      listener.release(frames);
//      continue;
//    }
//
//#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
//    viewer.addFrame("RGB", rgb);
//    viewer.addFrame("ir", ir);
//    viewer.addFrame("depth", depth);
//    viewer.addFrame("registered", &registered);
//
//    protonect_shutdown = protonect_shutdown || viewer.render();
//#endif
//
///// [loop end]
//    listener.release(frames);
//    /** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */
//  }
///// [loop end]
//
//  // TODO: restarting ir stream doesn't work!
//  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
///// [stop]
//  dev->stop();
//  dev->close();
///// [stop]
//
//  delete registration;
}


#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <string>
#include "vtkVRMLImporter.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkOBJImporter.h"
#include "vtkTestUtilities.h"
#include "vtkNew.h"
#include "vtkJPEGWriter.h"
#include "vtkPNGWriter.h"
#include "vtkImageCanvasSource2D.h"
#include "vtkImageCast.h"
#include "vtkCamera.h"


//#include "vtk3DSImporter.h"
#include <vtkDataSet.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

int vtkTest4(std::string filePath)
{
	// Parse command line arguments
	

	
	//importer->Update();
	//reader->Update();
	//vtkSmartPointer<vtkTexture> colorTexture = vtkSmartPointer<vtkTexture>::New();
	//colorTexture->SetInputConnection(reader->GetOutputPort());
	//colorTexture->InterpolateOn();

	// Visualize
	/*vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	*/

	//actor->SetTexture(colorTexture);
	
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	//renderer->AddActor(actor);
	//renderer->SetBackground(.3, .6, .3); // Background color green

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	//renderWindowInteractor->SetRenderWindow(importer->GetRenderWindow());

	std::string filename = filePath + "sibenik.obj";
	std::string filenameMTL = filePath + "sibenik.mtl";
	vtkSmartPointer<vtkOBJImporter> importer = vtkSmartPointer<vtkOBJImporter>::New();

	importer->SetFileName(filename.c_str());
	importer->SetFileNameMTL(filenameMTL.c_str());
	importer->SetTexturePath(filePath.c_str());
	importer->Read();
	importer->SetRenderWindow(renderWindow);
	importer->Update();

	//std::string filename = filePath;
	//vtkSmartPointer<vtkVRMLImporter> importer = vtkSmartPointer<vtkVRMLImporter>::New();
	//importer->SetFileName(filename.c_str());
	//importer->SetRenderWindow(renderWindow);
	//importer->Update();

	//std::string filename = filePath;
	//vtkSmartPointer<vtk3DSImporter> importer = vtkSmartPointer<vtk3DSImporter>::New();
	//importer->SetFileName(filename.c_str());
	//importer->SetRenderWindow(renderWindow);
	//importer->Update();

	
	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}




#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkGraphicsFactory.h>

int vtkTest6(std::string filePath)
{
	vtkObject::GlobalWarningDisplayOff();

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	//renderer->SetBackground(.3, .6, .3); // Background color green

	vtkSmartPointer<vtkCamera> camera =
		vtkSmartPointer<vtkCamera>::New();


	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();

	renderWindow->SetOffScreenRendering(1);
	renderWindow->AddRenderer(renderer);
	renderer->SetActiveCamera(camera);

	//vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	//	vtkSmartPointer<vtkRenderWindowInteractor>::New();
	//renderWindowInteractor->SetRenderWindow(renderWindow);
	//renderWindowInteractor->SetRenderWindow(importer->GetRenderWindow());

	std::string filename = filePath + "sibenik.obj";
	std::string filenameMTL = filePath + "sibenik.mtl";
	vtkSmartPointer<vtkOBJImporter> importer = vtkSmartPointer<vtkOBJImporter>::New();

	importer->SetFileName(filename.c_str());
	importer->SetFileNameMTL(filenameMTL.c_str());
	importer->SetTexturePath(filePath.c_str());
	importer->Read();
	importer->SetRenderWindow(renderWindow);
	importer->Update();
	

	renderWindow->Render();
	//renderWindowInteractor->Start();
	renderer->SetBackground(.3, .6, .3);
	double *a = camera->GetPosition();
	namedWindow("window", WINDOW_AUTOSIZE);
	
	
	for (int i = 1; i < 10; i += 1) {
		
		camera->SetPosition(*(a), *(a + 1), *(a + 2) + i);
		vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
			vtkSmartPointer<vtkWindowToImageFilter>::New();
		windowToImageFilter->SetInput(renderWindow);
		windowToImageFilter->Update();

		vtkImageData* image = windowToImageFilter->GetOutput();

		const int numComponents = image->GetNumberOfScalarComponents(); // 3 

																		//Construct the OpenCv Mat 
		int dims[3];
		image->GetDimensions(dims);
		cv::Mat openCVImage(dims[1], dims[0], CV_8UC3, image->GetScalarPointer()); // Unsigned int, 4 channels 

		//cvtColor(openCVImage, openCVImage, CV_BGRA2GRAY);

		// Flip because of different origins between vtk and OpenCV 
		cv::flip(openCVImage, openCVImage, 0);

		imshow("window", openCVImage);
		//resizeWindow("window", 500, 500);
		//moveWindow("window", 0, 0);

		waitKey(500);
	}
	destroyWindow("window");


	return EXIT_SUCCESS;
}

#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <vtkPlanes.h>
#include <vtkMapper.h>
#include <vtkCameraActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
 
int vtkTestC(std::string filePath)
{


	//std::string filename = filePath + ".obj";
	//std::string filenameMtl = filePath + ".mtl";

	std::string filename = filePath + "sibenik.obj";
	std::string filenameMtl = filePath + "sibenik.mtl";
	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();

	vtkSmartPointer<vtkOBJImporter> importer = vtkSmartPointer<vtkOBJImporter>::New();

	importer->SetFileName(filename.c_str());
	importer->SetFileNameMTL(filenameMtl.c_str());
	importer->SetTexturePath(filePath.c_str());
	reader->SetFileName(filename.c_str());
	reader->Update();
	
	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	
  // Camera
  vtkSmartPointer<vtkCamera> camera = 
    vtkSmartPointer<vtkCamera>::New();

  vtkSmartPointer<vtkCameraActor> cameraActor = 
    vtkSmartPointer<vtkCameraActor>::New();
  cameraActor->SetCamera(camera);
 
  // (Xmin,Xmax,Ymin,Ymax,Zmin,Zmax).
  double* bounds = new double[6];
  bounds = cameraActor->GetBounds();
  
  std::cout << "bounds: " << bounds[0] << " " << bounds[1] << " " << bounds[2] << " " << 
      bounds[3] << " " << bounds[4] << " " << bounds[5] << std::endl;
 
  // Visualize
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetOffScreenRendering(1);
  renderWindow->AddRenderer(renderer);
 
  //vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
  //  vtkSmartPointer<vtkRenderWindowInteractor>::New();
  //renderWindowInteractor->SetRenderWindow(renderWindow);
 
  actor->SetScale(100);
  renderer->AddActor(actor);
  //renderer->AddActor(cameraActor);
  renderer->SetActiveCamera(camera);
  //vtkProp3D* center = actor->GetCenter();
  double *a = actor->GetCenter();
  double *o = actor->GetOrientation();
  std::cout << *(a) << " " << *(a+1) << " " << *(a+2) << " " <<  "\n" << *(o) << " " << *(o + 1) << " " << *(o + 2) << "\n";
  //cameraActor->SetOrigin(actor->GetCenter());
  camera->SetPosition(*(a), *(a + 1), *(a + 2) + 300);
  //camera->SetFocalPoint(*(o), *(o + 1) + 100, -*(o + 2));
  camera->SetFocalPoint(*(a), *(a + 1), *(a + 2));
  //camera->ParallelProjectionOn();
  //camera->SetParallelScale(100);

  //actor->GetOrientation();
  //cameraActor->SetOrigin(0,0,-100);
  


  renderer->SetBackground(.3, .6, .3); // Background color white

  renderWindow->Render();
  namedWindow("window", WINDOW_AUTOSIZE);

  for (int i = 300; i < 1000; i += 100) {
	  camera->SetPosition(*(a), *(a + 1), *(a + 2) + i);

	  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
		  vtkSmartPointer<vtkWindowToImageFilter>::New();
	  windowToImageFilter->SetInput(renderWindow);
	  windowToImageFilter->Update();

	  vtkImageData* image = windowToImageFilter->GetOutput();

	  const int numComponents = image->GetNumberOfScalarComponents(); // 3 

	  //Construct the OpenCv Mat 
	  int dims[3];
	  image->GetDimensions(dims);
	  cv::Mat openCVImage(dims[1], dims[0], CV_8UC3, image->GetScalarPointer()); // Unsigned int, 4 channels 

	  cvtColor(openCVImage, openCVImage, CV_BGRA2GRAY);

	  // Flip because of different origins between vtk and OpenCV 
	  cv::flip(openCVImage, openCVImage, 0);

	  imshow("window", openCVImage);
	  //resizeWindow("window", 500, 500);
	  //moveWindow("window", 0, 0);

	  waitKey(1000);
  }
  destroyWindow("window");

 return EXIT_SUCCESS;
}


void PT::test1()
{
	//vtkTest4("C:\\Users\\Adam\\Desktop\\volumetric data\\cornell-box\\CornellBox-Original");
	//vtkTest4("C:\\Users\\Adam\\Desktop\\volumetric data\\sibenik\\");
	vtkTest6("C:\\Users\\Adam\\Desktop\\volumetric data\\sibenik\\");
	//vtkTestC("C:\\Users\\Adam\\Desktop\\volumetric data\\sibenik\\");
	//vtkTestC("C:\\Users\\Adam\\Desktop\\volumetric data\\cornell-box\\CornellBox-Original");
	
}


