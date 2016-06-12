#include "objObject.h"
#include "util.h"

#include <vtkTransform.h>

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
	prevOrient = cv::Vec3f(0, 0, 0);
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
	f = camera->GetViewUp();
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
	for (int i = 0; i < 3; i++)
		if ((int)(100 * orientation[i]) == -(int)(100 * prevOrient[i])) orientation[i] = -orientation[i];
	for (int i = 0; i < 3; i++) {
		if ((orientation[i]*prevOrient[i]) < 0 && abs(orientation[i])>0.5) orientation[i] = -orientation[i];
	}
	cv::Point3f oldCenter((*position), *(position + 1), *(position + 2));
	double dist = cv::norm(oldCenter - center);
	if (dist > 0.2) {
		camera->SetPosition(center.x, center.y, center.z);
	}

	float scale = 0.018;
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Identity();
	transform->Scale(scale, scale, scale);
	camera->ApplyTransform(transform);

	cv::Point3f focal;
	float s = 360;
	focal.x = orientation[0]*s;
	focal.z = -100.0;// orientation[1] * s;
	focal.y = (0.15 - orientation[1] )* s;

	double *foc = camera->GetFocalPoint();
	cv::Point3f oldFocal((*foc), *(foc + 1), *(foc + 2));
	cv::Point3f scaledFocal = focal * scale;

	focal.x = (int)focal.x - (int)focal.x % 5;
	focal.y = (int)focal.y - (int)focal.y % 5;
	camera->SetFocalPoint(focal.x, focal.y, (int)focal.z);
	double *f = camera->GetFocalPoint();
	
	if (*(f)*scale*oldFocal.x<0 && abs(oldFocal.x - *(f)*scale > 1.0)) focal.x = -focal.x;
	if (*(f + 1)*scale*oldFocal.y<0 && abs(oldFocal.y - *(f + 1)*scale > 2.0)) focal.y = -focal.y;


	camera->SetFocalPoint(focal.x, focal.y, (int)focal.z);

	f = camera->GetFocalPoint();
	
	//std::cout << "FOCAL POINT NEW: " << *(f)*scale << " " << *(f + 1)*scale << " " << *(f + 2)*scale << "\n";
	//std::cout << "FOCAL POINT OLD: " << oldFocal.x << " " << oldFocal.y << " " << oldFocal.z << "\n";

	prevOrient = orientation;


}
