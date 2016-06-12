
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkOBJImporter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


using namespace std;




class objObject
{
public:
	objObject(string _path, string _name);
	//~objObject();
	void loadData();
	cv::Mat render();
	void setCameraUp(double x, double y, double z);
	void moveCameraBy(double x, double y, double z);
	void rotateCameraBy(double x, double y, double z);
	void setCamera(cv::Point3f center, cv::Vec3f orientation);
	
private:
	string path;
	string name;
	double *position;
	cv::Vec3f prevOrient;
	vtkSmartPointer<vtkOBJImporter> reader;
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkCamera> camera;
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter;

};




