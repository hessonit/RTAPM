#include "objObject.h"
#include "util.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

objObject::objObject(string _path)
{
	path = _path;
}

void objObject::readData()
{
	string filename = path + ".obj";
	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
}
