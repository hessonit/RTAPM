
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


using namespace std;




class objObject
{
public:
	objObject(string _path, string _name);
	//~objObject();
	void loadData();
	void render();

private:
	string path;
	string name;
	vtkSmartPointer<vtkOBJImporter> reader;
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkCamera> camera;
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter;

};




