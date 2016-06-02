#include "objObject.h"
#include "util.h"

#include <vtkTransform.h>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>

objObject::objObject(string _path, string _name)
{
	path = _path;
	name = _name;

	renderer = vtkSmartPointer<vtkRenderer>::New();
	camera = vtkSmartPointer<vtkCamera>::New();
	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	//windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	renderWindow->SetOffScreenRendering(1);
	renderWindow->AddRenderer(renderer);
	renderer->SetActiveCamera(camera);
	//windowToImageFilter->SetInput(renderWindow);
	position = camera->GetPosition();
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
	double *f = camera->GetFocalPoint();
	std::cout << "FOCAL POINT: " << *(f) << " " << *(f + 1) << " " << *(f + 2) << "\n";
	f = camera->GetViewUp();
	std::cout << "VIEW UP: " << *(f) << " " << *(f + 1) << " " << *(f + 2) << "\n";
}

cv::Mat objObject::render()
{
	windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(renderWindow);
	windowToImageFilter->Update();

	vtkImageData* image = windowToImageFilter->GetOutput();
	const int numComponents = image->GetNumberOfScalarComponents();
	int dims[3];
	image->GetDimensions(dims);
	cv::Mat openCVImage(dims[1], dims[0], CV_8UC3, image->GetScalarPointer());
	//cvtColor(openCVImage, openCVImage, CV_BGRA2GRAY);
	cv::flip(openCVImage, openCVImage, 0);
	return openCVImage;
}

void objObject::setCameraUp(double x, double y, double z)
{
	
	camera->SetViewUp(x, y, z);
	
}

void objObject::moveCameraBy(double x, double y, double z)
{
	//double *pos = camera->GetPosition();
	camera->SetPosition((*position) + x, *(position + 1) + y, *(position + 2) + z);
}

void objObject::rotateCameraBy(double x, double y, double z)
{
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Identity();
	transform->RotateX(x);
	transform->RotateY(y);
	transform->RotateZ(z);
	

	camera->ApplyTransform(transform);
}

void objObject::setCamera(cv::Point3f center, cv::Vec3f orientation)
{
	cv::Point3f oldCenter((*position), *(position + 1), *(position + 2));
	double dist = cv::norm(oldCenter - center);
	if (dist > 0.02) {
		camera->SetPosition(center.x, center.y, center.z);
	}

	float scale = 2;
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Identity();
	transform->Scale(scale, scale, scale);
	camera->ApplyTransform(transform);

	double focus[3];
	double viewup[3];
	focus[0] = center.x - -cos(orientation[0])*sin(orientation[1]);
	focus[1] = center.y - sin(orientation[0]);
	focus[2] = center.z - cos(orientation[0])*cos(orientation[1]);

	float s = 25;
	focus[0] = orientation[0]*s;
	focus[1] = orientation[2]*s;
	focus[2] = orientation[1]*s;

	//viewup[0] = cos(orientation[1])*sin(orientation[2]) +
	//	sin(orientation[1])*sin(orientation[2])*cos(orientation[2]);
	//viewup[1] = cos(orientation[0])*cos(orientation[2]);
	//viewup[2] = sin(orientation[1])*sin(orientation[2]) -
	//	cos(orientation[1])*sin(orientation[0])*cos(orientation[2]);

	////set the camera position and orientation
	//cam->SetPosition(position);
	//camera->SetViewUp(viewup);

	//camera->SetFocalPoint(focus[0] + 30, focus[2] - 30, focus[1] + 15);
	
	camera->SetFocalPoint(-focus[0], focus[1], -focus[2]);
	double *f = camera->GetFocalPoint();
	std::cout << "FOCAL POINT NEW: " << *(f) << " " << *(f + 1) << " " << *(f + 2) << "\n";
	f = camera->GetViewUp();
	std::cout << "VIEW UP NEW: " << *(f) << " " << *(f + 1) << " " << *(f + 2) << "\n";


}
