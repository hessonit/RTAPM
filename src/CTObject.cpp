#include "CTObject.h"
#include "util.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>




void CTObject::readData()
{
	//_ASSERT(xSize > 0 && ySize > 0 && zSize > 0);
	std::cout << "Reading data\n";
	ifstream inFile;
	size_t size = 0;
	for (int j = start; j <= start+zSize; j++) {
		std::cout << j << "/" << zSize;
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
		std::cout << '\r';
	}
}

void CTObject::readDataPNG()
{
	std::cout << "Reading data\n";
	ifstream inFile;
	int size = 0;
	size = xSize * ySize;
	for (int j = start; j < start + zSize; j++) {
		std::cout << j-start+1 << "/" << zSize;
		int ind = j-start;

		cv::Mat image = cv::imread(path + intToStr(j) + ".png", CV_LOAD_IMAGE_GRAYSCALE);
		data[ind] = new char[size];
		for (int i = 0; i < image.size[0]; i++)
		{
			for (int k = 0; k < image.size[1]; k++)
			{
				data[ind][k*ySize + i] = image.at<char>(k, i);
			}
		}

		std::cout << '\r';
	}
}

void CTObject::showData()
{
	cv::namedWindow("showData", CV_WINDOW_NORMAL);
	cv::moveWindow("showData", 1200, 0);
	cv::setWindowProperty("showData", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	for (int i = 0; i < zSize; i++)
	{
		cv::Mat image = cv::Mat(xSize, ySize, CV_8UC1, data[i]);
		//setWindowProperty("showData", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		cv::imshow("showData", image);
		int op = cv::waitKey(50);
	}
}
void CTObject::getCorrectDimensions(int* x, int* y, int*z) 
{
	switch (view)
	{
		case FRONT:
			*x = xSize; *y = ySize; *z = zSize;
			break;
		case BACK:
			*x = xSize; *y = ySize; *z = zSize;
			break;
		case LEFT:
			*x = zSize; *y = ySize; *z = xSize;
			break;
		case RIGHT:
			*x = zSize; *y = ySize; *z = xSize;
			break;
		case TOP:
			*x = xSize; *y = zSize; *z = ySize;
			break;
		case BOTTOM:
			*x = xSize; *y = zSize; *z = ySize;
			break;
		default:
			*x = xSize; *y = ySize; *z = zSize;
			break;
	}
}
//Mat ROI = board(Rect(i, j, 1, 1));
//char value = ctObject.at(xVal, yVal, zVal);
//ROI.setTo(Scalar(value, value, value, 100));
void CTObject::showDataFromView()
{
	cv::namedWindow("showData", CV_WINDOW_NORMAL);
	cv::moveWindow("showData", 0, 0);
	int x, y, z;
	getCorrectDimensions(&x, &y, &z);
	for (int i = 0; i < z; i++)
	{
		cv::Mat image = cv::Mat(y, x, CV_8UC1, cv::Scalar(0));
		//setWindowProperty("showData", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		for (int xx = 0; xx < x; xx++)
		{
			for (int yy = 0; yy < y; yy++)
			{
				cv::Mat ROI = image(cv::Rect(xx, yy, 1, 1));
				char value = at(xx, yy, i);
				ROI.setTo(value);
			}
		}
		cv::imshow("showData", image);
		int op = cv::waitKey(50);
	}
}

// TO-DO check if everything works
char CTObject::at(int x, int y, int z)
{
	//_ASSERT(xSize > x && ySize > y && zSize > z);
	if ((xSize <= x || ySize <= y || zSize <= z) || (0 > x || 0 > y || 0 > z))
	{
		//std::cout << "outside\n";
		return data[0][0];
	}
	switch (view)
	{
	case FRONT:
		return data[z][y*xSize + x];
		break;
	case BACK:
		return data[zSize - z - 1][y*ySize + x];
		break;
	case LEFT:
		return data[zSize - x - 1][y*ySize + z];
		break;
	case RIGHT:
		return data[x][y*ySize + (xSize - z - 1)];
		break;
	case TOP:
		return data[zSize - y - 1][z*xSize + x];
		break;
	case BOTTOM:
		return data[y][(ySize - z)*ySize + x];
		break;
	default:
		return 0;
		break;
	}
}




