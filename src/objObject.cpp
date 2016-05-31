#include "objObject.h"
#include "util.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

objObject::objObject(string _path, string _name)
{
	path = _path;
	name = _name;

	renderer = vtkSmartPointer<vtkRenderer>::New();
	camera = vtkSmartPointer<vtkCamera>::New();
	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	renderWindow->SetOffScreenRendering(1);
	renderWindow->AddRenderer(renderer);
	renderer->SetActiveCamera(camera);
	windowToImageFilter->SetInput(renderWindow);
}

void objObject::loadData()
{
	vtkObject::GlobalWarningDisplayOff();
	

	renderWindow->SetOffScreenRendering(1);
	renderWindow->AddRenderer(renderer);
	renderer->SetActiveCamera(camera);

	string filenameOBJ = path + name + ".obj";
	string filenameMTL = path + name + ".mtl";

	reader = vtkSmartPointer<vtkOBJImporter>::New();
	reader->SetFileName(filenameOBJ.c_str());
	reader->SetFileNameMTL(filenameMTL.c_str());
	reader->SetTexturePath(path.c_str());
	reader->SetRenderWindow(renderWindow);
	reader->Read();

}

void objObject::render()
{
	windowToImageFilter->Update();

	vtkImageData* image = windowToImageFilter->GetOutput();
	const int numComponents = image->GetNumberOfScalarComponents();
	int dims[3];
	image->GetDimensions(dims);
	cv::Mat openCVImage(dims[1], dims[0], CV_8UC3, image->GetScalarPointer());
	//cvtColor(openCVImage, openCVImage, CV_BGRA2GRAY);
	cv::flip(openCVImage, openCVImage, 0);

}
