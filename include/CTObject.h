#include <fstream>
#include <cstring> // for std::strlen
#include <cstddef> // for std::size_t -> is a typedef on an unsinged int
#include <cstdint>
#include <iostream>

using namespace std;
class CTObject
{
public:
	CTObject(string _path, int _xSize, int _ySize, int _zSize);
	~CTObject();
	void readData();
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

CTObject::~CTObject()
{
	for (int i = 0; i < zSize; ++i) {
		delete[] data[i];
	}
	delete[] data;
}