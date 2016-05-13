#include <fstream>
#include <cstring>  
#include <cstddef>
#include <cstdint>
#include <iostream>

using namespace std;

enum SIDE{FRONT, BACK, LEFT, RIGHT, TOP, BOTTOM};

class CTObject
{
public:
	CTObject(string _path, int _xSize, int _ySize, int _zSize);
	~CTObject();
	void readData();
	void showData();
	void showDataFromView();
	char at(int x, int y, int z);
	int getWidth () { return xSize; }
	int getHeight() { return ySize; }
	int getDepth () { return zSize; }
	void setView(SIDE newView) { view = newView; }
private:
	void getCorrectDimensions(int* x, int* y, int*z);
	SIDE view;
	char **data;
	string path;
	const int xSize, ySize, zSize;
};


inline CTObject::CTObject(string _path, int _xSize, int _ySize, int _zSize) : xSize(_xSize), ySize(_ySize), zSize(_zSize)
{
	path = _path;
	data = new char*[_zSize];
	view = SIDE::FRONT;
}

inline CTObject::~CTObject()
{
	if (data != NULL) {
		for (int i = 0; i < zSize; ++i) {
			delete[] data[i];
		}
		delete[] data;
	}
}