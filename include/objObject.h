
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkOBJReader.h>
#include <string>


using namespace std;




class objObject
{
public:
	objObject(string _path);
	//~objObject();
	void readData();

private:
	string path;
	vtkSmartPointer<vtkOBJReader> reader;

};




