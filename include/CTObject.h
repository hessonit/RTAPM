#include <fstream>
#include <cstring>  
#include <cstddef>
#include <cstdint>
#include <iostream>

using namespace std;
class CTObject
{
public:
	CTObject(string _path, int _xSize, int _ySize, int _zSize);
	~CTObject();
	void readData();
	void showData();
	int getWidth () { return xSize; }
	int getHeight() { return ySize; }
	int getDepth () { return zSize; }
private:
	char **data;
	string path;
	const int xSize, ySize, zSize;
};


inline CTObject::CTObject(string _path, int _xSize, int _ySize, int _zSize) : xSize(_xSize), ySize(_ySize), zSize(_zSize)
{
	path = _path;
	data = new char*[_zSize];
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