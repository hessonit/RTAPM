#include "CTObject.h"
#include "util.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>




void CTObject::readData()
{
	_ASSERT(xSize > 0 && ySize > 0 && zSize > 0);
	ifstream inFile;
	size_t size = 0;
	for (int j = 1; j <= zSize; j++) {
		int ind = j;

		inFile.open(path + intToStr(ind), ios::in | ios::binary | ios::ate);
		char* oData = 0;
		inFile.seekg(0, ios::end);
		size = inFile.tellg();
		inFile.seekg(0, std::ios::beg);

		oData = new char[size + 1]; 
		data[j - 1] = new char[size / 2];
		inFile.read(oData, size);
		int mini = 0;//  9999999; // 0 65535
		int maxi = 63536;//  0;
		int mini2 = 9999999; // 0 65535
		int maxi2 = 0;
		/*for (int i = 0; i < size; i += 2) {
		short c = (((short)oData[i]) << 8) | oData[i+1];
		if (c < mini2) mini2 = c;
		if (c > maxi2) maxi2 = c;
		}
		std::cout << "\nmini: " << mini2 << " maxi: " << maxi2 << "\n";
		*/
		mini = -2000;
		maxi = 2531 + 2000;
		for (int i = 0; i < size; i += 2) {
			short c = (((short)oData[i]) << 8) | oData[i + 1];
			float temp = (float)c;
			temp = ((temp - mini) / ((float)maxi)) * 255;
			//if (temp > 40) temp = 200;
			data[j-1][i / 2] = (char)temp;
			/*char value = 0;
			if (c > -700) value += 31;
			if (c > -300) value += 31;
			if (c > -100) value += 31;
			if (c > 0) value += 31;
			if (c > 15) value += 31;
			if (c > 30) value += 31;
			if (c > 45) value += 31;
			if (c > 700) value += 31;
			if (c > 1000) value += 31;
			image[i / 2] = value;*/
		}
		inFile.close();
		delete[] oData;
	}
}

void CTObject::showData()
{
	cv::namedWindow("showData", CV_WINDOW_NORMAL);
	cv::moveWindow("showData", 0, 0);
	for (int i = 0; i < zSize; i++)
	{
		cv::Mat bunny = cv::Mat(xSize, ySize, CV_8UC1, data[i]);
		//setWindowProperty("showData", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		cv::imshow("showData", bunny);
		int op = cv::waitKey(50);
	}
}




