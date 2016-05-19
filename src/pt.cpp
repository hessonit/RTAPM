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
//#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
  // viewer_enabled = false;
  // SimpleViewer viewer; 
  SimpleViewer viewer;
  if (viewer_enabled)
    viewer.initialize();
//#else
  //viewer_enabled = false;
//#endif

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



//
//int vtkTest()
//{
//	// Create a sphere
//	vtkSmartPointer<vtkSphereSource> sphereSource =
//		vtkSmartPointer<vtkSphereSource>::New();
//	sphereSource->SetCenter(0.0, 0.0, 0.0);
//	sphereSource->SetRadius(5.0);
//	sphereSource->Update();
//
//	// Create a mapper
//	vtkSmartPointer<vtkPolyDataMapper> mapper =
//		vtkSmartPointer<vtkPolyDataMapper>::New();
//#if VTK_MAJOR_VERSION <= 5
//	mapper->SetInput(sphereSource->GetOutput());
//#else
//	mapper->SetInputData(sphereSource->GetOutput());
//#endif
//
//	// Create an actor
//	vtkSmartPointer<vtkActor> actor =
//		vtkSmartPointer<vtkActor>::New();
//	actor->SetMapper(mapper);
//
//	// Create a renderer
//	vtkSmartPointer<vtkRenderer> renderer =
//		vtkSmartPointer<vtkRenderer>::New();
//	renderer->SetBackground(1, 1, 1); // Set background color to white
//	renderer->AddActor(actor);
//
//	// Create a render window
//	vtkSmartPointer<vtkRenderWindow> renderWindow =
//		vtkSmartPointer<vtkRenderWindow>::New();
//	renderWindow->AddRenderer(renderer);
//
//	// Create an interactor
//	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//		vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	renderWindowInteractor->SetRenderWindow(renderWindow);
//
//	// Setup the text and add it to the renderer
//	vtkSmartPointer<vtkTextActor> textActor =
//		vtkSmartPointer<vtkTextActor>::New();
//	textActor->SetInput("Hello world");
//	textActor->SetPosition2(10, 40);
//	textActor->GetTextProperty()->SetFontSize(24);
//	textActor->GetTextProperty()->SetColor(1.0, 0.0, 0.0);
//	renderer->AddActor2D(textActor);
//
//	// Render and interact
//	renderWindow->Render();
//	renderWindowInteractor->Start();
//
//	return EXIT_SUCCESS;
//}
//
//void CreateImageData(vtkImageData* imageData)
//{
//	// Create a spherical implicit function.
//	vtkSmartPointer<vtkSphere> sphere =
//		vtkSmartPointer<vtkSphere>::New();
//	sphere->SetRadius(0.1);
//	sphere->SetCenter(0.0, 0.0, 0.0);
//
//	vtkSmartPointer<vtkSampleFunction> sampleFunction =
//		vtkSmartPointer<vtkSampleFunction>::New();
//	sampleFunction->SetImplicitFunction(sphere);
//	sampleFunction->SetOutputScalarTypeToDouble();
//	sampleFunction->SetSampleDimensions(127, 127, 127); // intentional NPOT dimensions.
//	sampleFunction->SetModelBounds(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
//	sampleFunction->SetCapping(false);
//	sampleFunction->SetComputeNormals(false);
//	sampleFunction->SetScalarArrayName("values");
//	sampleFunction->Update();
//
//	vtkDataArray* a = sampleFunction->GetOutput()->GetPointData()->GetScalars("values");
//	double range[2];
//	a->GetRange(range);
//
//	vtkSmartPointer<vtkImageShiftScale> t =
//		vtkSmartPointer<vtkImageShiftScale>::New();
//	t->SetInputConnection(sampleFunction->GetOutputPort());
//
//	t->SetShift(-range[0]);
//	double magnitude = range[1] - range[0];
//	if (magnitude == 0.0)
//	{
//		magnitude = 1.0;
//	}
//	t->SetScale(255.0 / magnitude);
//	t->SetOutputScalarTypeToUnsignedChar();
//
//	t->Update();
//
//	imageData->ShallowCopy(t->GetOutput());
//}
//int vtkTest2()
//{
//	vtkSmartPointer<vtkImageData> imageData =
//		vtkSmartPointer<vtkImageData>::New();
//	//if (argc < 2)
//	//{
//		CreateImageData(imageData);
//	//}
//	//else
//	//{
//	//	vtkSmartPointer<vtkXMLImageDataReader> reader =
//	//		vtkSmartPointer<vtkXMLImageDataReader>::New();
//	//	reader->SetFileName(argv[1]);
//	//	reader->Update();
//	//	imageData->ShallowCopy(reader->GetOutput());
//	//}
//
//	vtkSmartPointer<vtkRenderWindow> renWin =
//		vtkSmartPointer<vtkRenderWindow>::New();
//	vtkSmartPointer<vtkRenderer> ren1 =
//		vtkSmartPointer<vtkRenderer>::New();
//	ren1->SetBackground(0.1, 0.4, 0.2);
//
//	renWin->AddRenderer(ren1);
//
//	renWin->SetSize(301, 300); // intentional odd and NPOT  width/height
//
//	vtkSmartPointer<vtkRenderWindowInteractor> iren =
//		vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	iren->SetRenderWindow(renWin);
//
//	renWin->Render(); // make sure we have an OpenGL context.
//
//	vtkSmartPointer<vtkSmartVolumeMapper> volumeMapper =
//		vtkSmartPointer<vtkSmartVolumeMapper>::New();
//	volumeMapper->SetBlendModeToComposite(); // composite first
//#if VTK_MAJOR_VERSION <= 5
//	volumeMapper->SetInputConnection(imageData->GetProducerPort());
//#else
//	volumeMapper->SetInputData(imageData);
//#endif  
//	vtkSmartPointer<vtkVolumeProperty> volumeProperty =
//		vtkSmartPointer<vtkVolumeProperty>::New();
//	volumeProperty->ShadeOff();
//	volumeProperty->SetInterpolationType(VTK_LINEAR_INTERPOLATION);
//
//	vtkSmartPointer<vtkPiecewiseFunction> compositeOpacity =
//		vtkSmartPointer<vtkPiecewiseFunction>::New();
//	compositeOpacity->AddPoint(0.0, 0.0);
//	compositeOpacity->AddPoint(80.0, 1.0);
//	compositeOpacity->AddPoint(80.1, 0.0);
//	compositeOpacity->AddPoint(255.0, 0.0);
//	volumeProperty->SetScalarOpacity(compositeOpacity); // composite first.
//
//	vtkSmartPointer<vtkColorTransferFunction> color =
//		vtkSmartPointer<vtkColorTransferFunction>::New();
//	color->AddRGBPoint(0.0, 0.0, 0.0, 1.0);
//	color->AddRGBPoint(40.0, 1.0, 0.0, 0.0);
//	color->AddRGBPoint(255.0, 1.0, 1.0, 1.0);
//	volumeProperty->SetColor(color);
//
//	vtkSmartPointer<vtkVolume> volume =
//		vtkSmartPointer<vtkVolume>::New();
//	volume->SetMapper(volumeMapper);
//	volume->SetProperty(volumeProperty);
//	ren1->AddViewProp(volume);
//	ren1->ResetCamera();
//
//	// Render composite. In default mode. For coverage.
//	renWin->Render();
//
//	// 3D texture mode. For coverage.
//#if !defined(VTK_LEGACY_REMOVE)
//	volumeMapper->SetRequestedRenderModeToRayCastAndTexture();
//#endif // VTK_LEGACY_REMOVE
//	renWin->Render();
//
//	// Software mode, for coverage. It also makes sure we will get the same
//	// regression image on all platforms.
//	volumeMapper->SetRequestedRenderModeToRayCast();
//	renWin->Render();
//
//	iren->Start();
//
//	return EXIT_SUCCESS;
//}
//
//#include <vtkEarthSource.h>
//#include <vtkPolyData.h>
//#include <vtkSmartPointer.h>
//#include <vtkPolyDataMapper.h>
//#include <vtkActor.h>
//#include <vtkRenderWindow.h>
//#include <vtkRenderer.h>
//#include <vtkRenderWindowInteractor.h>
//
//int vtkTest3()
//{
//
//	//Create a sphere
//	vtkSmartPointer<vtkEarthSource> earthSource = vtkSmartPointer<vtkEarthSource>::New();
//	earthSource->OutlineOff();
//	earthSource->Update();
//
//	//Create a mapper and actor
//	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//	mapper->SetInputConnection(earthSource->GetOutputPort());
//
//	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//	actor->SetMapper(mapper);
//
//	//Create a renderer, render window, and interactor
//	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
//	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
//	renderWindow->AddRenderer(renderer);
//	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	renderWindowInteractor->SetRenderWindow(renderWindow);
//
//	//Add the actor to the scene
//	renderer->AddActor(actor);
//	renderer->SetBackground(1, 1, 1); // Background color white
//
//									  //Render and interact
//	renderWindow->Render();
//	renderWindowInteractor->Start();
//
//	return EXIT_SUCCESS;
//}
//
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

int vtkTest4(std::string filePath)
{
	// Parse command line arguments
	

	std::string filename = filePath + ".obj";
	std::string filenameMTL = filePath + ".mtl";
	//vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();

	reader->SetFileName(filename.c_str());
	//reader->SetFileNameMTL(filenameMTL.c_str());
	reader->Update();

	//vtkSmartPointer<vtkTexture> colorTexture = vtkSmartPointer<vtkTexture>::New();
	//colorTexture->SetInputConnection(reader->GetOutputPort());
	//colorTexture->InterpolateOn();

	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	

	//actor->SetTexture(colorTexture);
	
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(actor);
	renderer->SetBackground(.3, .6, .3); // Background color green

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkInteractorStyleTrackballCamera.h>

int vtkTest5()
{
	// Sphere 1
	vtkSmartPointer<vtkSphereSource> sphereSource1 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource1->SetCenter(0.0, 0.0, 0.0);
	sphereSource1->SetRadius(4.0);
	sphereSource1->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper1 =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper1->SetInputConnection(sphereSource1->GetOutputPort());

	vtkSmartPointer<vtkActor> actor1 =
		vtkSmartPointer<vtkActor>::New();
	actor1->SetMapper(mapper1);

	// Sphere 2
	vtkSmartPointer<vtkSphereSource> sphereSource2 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource2->SetCenter(10.0, 0.0, 0.0);
	sphereSource2->SetRadius(3.0);
	sphereSource2->Update();

	// Create a mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper2 =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper2->SetInputConnection(sphereSource2->GetOutputPort());

	// Create an actor
	vtkSmartPointer<vtkActor> actor2 =
		vtkSmartPointer<vtkActor>::New();
	actor2->SetMapper(mapper2);

	// A renderer and render window
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	// An interactor
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	// Add the actors to the scene
	renderer->AddActor(actor1);
	renderer->AddActor(actor2);
	renderer->SetBackground(1, 1, 1); // Background color white

									  // Render an image (lights and cameras are created automatically)
	renderWindow->Render();

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
		vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

	renderWindowInteractor->SetInteractorStyle(style);

	// Begin mouse interaction
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkGraphicsFactory.h>
//#include <vtkImagingFactory.h>

int vtkTest6()
{
	// Setup offscreen rendering
	vtkSmartPointer<vtkGraphicsFactory> graphics_factory =
		vtkSmartPointer<vtkGraphicsFactory>::New();
	graphics_factory->SetOffScreenOnlyMode(1);
	graphics_factory->SetUseMesaClasses(1);

	//vtkSmartPointer<vtkImagingFactory> imaging_factory =
	//	vtkSmartPointer<vtkImagingFactory>::New();
	//imaging_factory->SetUseMesaClasses(1);

	// Create a sphere
	vtkSmartPointer<vtkSphereSource> sphereSource =
		vtkSmartPointer<vtkSphereSource>::New();

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(sphereSource->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// A renderer and render window
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetOffScreenRendering(1);
	renderWindow->AddRenderer(renderer);

	// Add the actors to the scene
	renderer->AddActor(actor);
	renderer->SetBackground(.1, .1, .1); // Background color white

	renderWindow->Render();
	//renderWindow->GetOffScreenRendering();
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
		vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(renderWindow);
	windowToImageFilter->Update();
	
	vtkSmartPointer<vtkPNGWriter> writer =
		vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName("screenshot.png");
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Write();

	return EXIT_SUCCESS;
}

void PT::test1()
{
	//vtkTest4("C:\\Users\\Adam\\Desktop\\volumetric data\\cornell-box\\CornellBox-Original");
	vtkTest6();
	
}


