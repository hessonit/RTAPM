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
  std::string program_path(argv[0]);
  std::cerr << "Environment variables: LOGFILE=<protonect.log>" << std::endl;
  std::cerr << "Usage: " << program_path << " [gl | cl | cpu] [<device serial>] [-noviewer]" << std::endl;
  size_t executable_name_idx = program_path.rfind("Master");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
  // avoid flooing the very slow Windows console with debug messages
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));
#else
  // create a console logger with debug level (default is console logger with info level)
/// [logging]
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
/// [logging]
#endif
/// [file logging]
  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
  if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);
/// [file logging]

/// [context]
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
/// [context]

/// [discovery]
  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();
/// [discovery]

  bool viewer_enabled = true;

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg(argv[argI]);

    if(arg == "cpu")
    {
      if(!pipeline)
/// [pipeline]
        pipeline = new libfreenect2::CpuPacketPipeline();
/// [pipeline]
    }
    else if(arg == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
    {
      serial = arg;
    }
    else if(arg == "-noviewer")
    {
      viewer_enabled = false;
    }
    else
    {
      std::cout << "Unknown argument: " << arg << std::endl;
    }
  }

  if(pipeline)
  {
/// [open]
    dev = freenect2.openDevice(serial, pipeline);
/// [open]
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return;
  }

  //signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

/// [listeners]
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
/// [listeners]

/// [start]
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
/// [start]

/// [registration setup]
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
/// [registration setup]

  size_t framecount = 0;
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
  // viewer_enabled = false;
  // SimpleViewer viewer; 
  Viewer viewer;
  if (viewer_enabled)
    viewer.initialize();
#else
  viewer_enabled = false;
#endif

bool guard = true;
/// [loop start]
  while(!protonect_shutdown)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
/// [loop start]

/// [registration]
    registration->apply(rgb, depth, &undistorted, &registered);
/// [registration]

    if(guard)
    {
      // Mat temp = frameToMat("depth", depth);
      // showMat(temp);
      // temp = frameToMat("registered", &undistorted);
      // showMat(temp);
      // temp = frameToMat("registered", &registered);
      // showMat(temp);
      // std::cout<<temp<<"\n";
    //   vector<Point2f> output;
    //   Size patternsize(5,4);
    //   Mat image2 = frameToMat("rgb", rgb);
    //   Mat image;
    //   cvtColor(image2, image, CV_BGR2GRAY);
    //   printf("/////////////////////////////////////////////////////////////////////LOOKING?????????????????????\n");
    //   bool patternfound = findChessboardCorners( image, patternsize, output,
    // CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
    //     + CALIB_CB_FAST_CHECK);

    //   if(patternfound){
    //   printf("/////////////////////////////////////////////////////////////////////found\n");
    //   // return true;
    //   cornerSubPix(image, output, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    

    //   drawChessboardCorners(image, patternsize, Mat(output), patternfound);

    //   showMat(image);


    // } else {
    //   printf("///////////////////////////////////////////////////////////////NOT found\n");
    //   // showMat(image);
    //   // return false;
    // }


      guard = false;
    }

    

    framecount++;
    if (!viewer_enabled)
    {
      if (framecount % 100 == 0)
        std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
      listener.release(frames);
      continue;
    }

#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
    viewer.addFrame("RGB", rgb);
    viewer.addFrame("ir", ir);
    viewer.addFrame("depth", depth);
    viewer.addFrame("registered", &registered);

    protonect_shutdown = protonect_shutdown || viewer.render();
#endif

/// [loop end]
    listener.release(frames);
    /** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */
  }
/// [loop end]

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
/// [stop]
  dev->stop();
  dev->close();
/// [stop]

  delete registration;
}




int vtkTest()
{
	// Create a sphere
	vtkSmartPointer<vtkSphereSource> sphereSource =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetCenter(0.0, 0.0, 0.0);
	sphereSource->SetRadius(5.0);
	sphereSource->Update();

	// Create a mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(sphereSource->GetOutput());
#else
	mapper->SetInputData(sphereSource->GetOutput());
#endif

	// Create an actor
	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// Create a renderer
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->SetBackground(1, 1, 1); // Set background color to white
	renderer->AddActor(actor);

	// Create a render window
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	// Create an interactor
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkTextActor> textActor =
		vtkSmartPointer<vtkTextActor>::New();
	textActor->SetInput("Hello world");
	textActor->SetPosition2(10, 40);
	textActor->GetTextProperty()->SetFontSize(24);
	textActor->GetTextProperty()->SetColor(1.0, 0.0, 0.0);
	renderer->AddActor2D(textActor);

	// Render and interact
	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}




// Test pcl model (Test1)
void PT::test1()
{
	vtkTest();
  //// std::cout<<"DO THE JOB\n";
  //processor freenectprocessor = OPENGL;
  //std::vector<int> ply_file_indices;
  //freenectprocessor = static_cast<processor>(2);


  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  //K2G k2g(freenectprocessor);
  //std::cout << "getting cloud" << std::endl;
  //cloud = k2g.getCloud();

  //cloud->sensor_orientation_.w() = 0.0;
  //cloud->sensor_orientation_.x() = 1.0;
  //cloud->sensor_orientation_.y() = 0.0;
  //cloud->sensor_orientation_.z() = 0.0;

  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  //PlySaver ps(cloud, false, false);
  //viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

  //bool done = false;
  //while((!viewer->wasStopped()) && (!done)){

  //  viewer->spinOnce ();
  //  std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

  //  cloud = k2g.updateCloud(cloud);

  //  std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
  //  std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
  //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  //  viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");      

  //}

  //k2g.shutDown();
}


