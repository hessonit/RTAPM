#include "Projector.h"
#include "CTObject.h"
#include "util.h"
#include "viewer.h"

#include <iostream>
#include <queue>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

void Projector::setMatrices(cv::Mat mt, cv::Mat mr, cv::Mat cam, cv::Mat pro)
{
	_mt = mt;
	_mr = mr;
	_cam = cam;
	_pro = pro;
}

void Projector::setKinect(libfreenect2::SyncMultiFrameListener * listener, libfreenect2::Freenect2Device * dev)
{
	_listener = listener;
	_dev = dev;
}

void Projector::ctProjection(std::string ctFilePath, int xDim, int yDim, int zDim)
{
	CTObject ctObject(ctFilePath, xDim, yDim, zDim);
	ctObject.readData();

	libfreenect2::FrameMap frames;
	libfreenect2::Registration* registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	bool shutdown = false;
	cv::namedWindow("CTviewer", CV_WINDOW_NORMAL);
	cv::moveWindow("CTviewer", 0, 0);
	//cv::setWindowProperty("CTviewer", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	while (!shutdown)
	{

		(_listener)->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		cv::Mat frame = frameToMat("registered", &registered);
		cv::Mat depthFrame = frameToMat("depth", depth);
		cv::Mat board(424, 512, CV_8UC4, cv::Scalar::all(0));
		int centerX = 512 / 2;
		int centerY = 242 / 2;
		depthFrame *= 0.001;
		for (int i = 0; i < 512; i++)
		{
			for (int j = 0; j < 424; j++)
			{
				float x = 0, y = 0, z = 0, color = 0;
				registration->getPointXYZRGB(&undistorted, &registered,
					j, i,
					x, y, z, color);
				z = depthFrame.at<float>(j, i);

				// TO-DO add size modification
				// TO-DO add projection
				if (z >= 0.5 && z <= 1.0 && x >= -0.2 && x <= 0.2 && y >= -0.2 && y <= 0.2) {

					int xVal = (x + 0.2)*(512.0 / 0.4);
					int yVal = (y + 0.2)*(512.0 / 0.4);
					int zVal = (z - 0.5)*(359.0 / 0.5);
					cv::Mat ROI = board(cv::Rect(i, j, 1, 1));
					char value = ctObject.at(xVal, yVal, zVal);
					ROI.setTo(cv::Scalar(value, value, value, 100));
				}
			}
		}
		cv::imshow("CTviewer", board);
		int op = cv::waitKey(1);
		if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
		{
			shutdown = true;
			cv::destroyWindow("CTviewer");
		}
		_listener->release(frames);
	}


}

void Projector::reproject(bool gpuView)
{
	libfreenect2::Registration* registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	libfreenect2::FrameMap frames;
	SimpleViewer viewer;
	bool shutdown = false;
	cv::Mat board(480, 640, CV_8UC4, cv::Scalar::all(255));
	if (!gpuView) {
		cv::namedWindow("reprojection", CV_WINDOW_NORMAL);
		cv::moveWindow("reprojection", 0, 0);
		//setWindowProperty("reprojection", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	}
	else {
		viewer.setSize(480, 640); // TO-DO change resolution
		viewer.initialize();
		libfreenect2::Frame b(640, 480, 4);
		b.data = board.data;
		viewer.addFrame("RGB", &b);
		shutdown = shutdown || viewer.render();
	}
	while (!shutdown)
	{
		board = cv::Mat(480, 640, CV_8UC4, cv::Scalar::all(255));
		std::vector<cv::Point3f> wrldSrc;
		if (!gpuView) cv::imshow("reprojection", board);
		(_listener)->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		for (int i = 0; i<512; i++)
		{
			for (int j = 0; j<424; j++)
			{
				float x = 0, y = 0, z = 0, color = 0;
				registration->getPointXYZRGB(&undistorted, &registered,
					i, j,
					x, y, z, color);

				if (z>0.5 && z<1.7)
				{
					x = static_cast<float>(x + right / ((double)640.0)); //////////TO-DO fix that
					y = static_cast<float>(y + up / ((double)480.0));

					x -= 0.5;
					y -= 0.5;
					double PI = 3.14159265;
					x = static_cast<float>(std::cos(rotX * PI / 180) * x - std::sin(rotX * PI / 180) * y);
					y = static_cast<float>(std::sin(rotX * PI / 180) * x + std::cos(rotX * PI / 180) * y);

					x += 0.5;
					y += 0.5;

					wrldSrc.push_back(cv::Point3f(x * 100,
						y * 100,
						z * 100));
				}
			}
		}
		
		if (wrldSrc.size() > 0) {
			std::vector<cv::Point2f> projected = projectPoints(wrldSrc);
			for (int i = 0; i < projected.size(); i++)
			{
				if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {

					cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
					ROI.setTo(cv::Scalar(100, 100, 150, 100));
				}
			}
			if (!gpuView) imshow("reprojection", board);
			else {
				libfreenect2::Frame b(640, 480, 4);
				b.data = board.data;
				viewer.addFrame("RGB", &b);
				shutdown = shutdown || viewer.render();
			}
		}
		(_listener)->release(frames);
		if (!gpuView) {
			int op = cv::waitKey(50);
			if (op == 100 || (char)(op) == 'd') right -= 1;
			if (op == 115 || (char)(op) == 's') up += 1;
			if (op == 97 || (char)(op) == 'a') right += 1;
			if (op == 119 || (char)(op) == 'w') up -= 1;

			if (op == 114 || (char)(op) == 'r') rotX -= 0.5;
			if (op == 102 || (char)(op) == 'f') rotX += 0.5;

			if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
			{
				std::cout << "right = " << right << ";\nup = " << up << ";\nrotX = " << rotX << ";\n";
				break;
			}
		}
		else {
			right = viewer.offsetX;
			up = viewer.offsetY;
			rotX = viewer.rot;
		}
	}
	if (!gpuView) cv::destroyWindow("reprojection");
	else {
		viewer.stopWindow();
	}
}

void Projector::reprojectPlane(bool gpuView)
{
	libfreenect2::Registration* registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	libfreenect2::FrameMap frames;
	SimpleViewer viewer;
	bool shutdown = false;
	cv::Mat board(480, 640, CV_8UC4, cv::Scalar::all(255));
	if (!gpuView) {
		cv::namedWindow("reprojection", CV_WINDOW_NORMAL);
		cv::moveWindow("reprojection", 0, 0);
		//setWindowProperty("reprojection", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	}
	else {
		viewer.setSize(480, 640); // TO-DO change resolution
		viewer.initialize();
		libfreenect2::Frame b(640, 480, 4);
		b.data = board.data;
		viewer.addFrame("RGB", &b);
		shutdown = shutdown || viewer.render();
	}
	while (!shutdown)
	{
		board = cv::Mat(480, 640, CV_8UC4, cv::Scalar::all(255));
		std::vector<cv::Point3f> wrldSrc;
		std::vector<cv::Point3f> plnSrc;
		if (!gpuView) cv::imshow("reprojection", board);
		(_listener)->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		plnSrc = findPlane(registration, &undistorted, &registered);
		for (int i = 0; i<512; i++)
		{
			for (int j = 0; j<424; j++)
			{
				float x = 0, y = 0, z = 0, color = 0;
				registration->getPointXYZRGB(&undistorted, &registered,
					i, j,
					x, y, z, color);

				if (z>0.5 && z<1.7)
				{
					x = static_cast<float>(x + right / ((double)640.0)); //////////TO-DO fix that
					y = static_cast<float>(y + up / ((double)480.0));

					x -= 0.5;
					y -= 0.5;
					double PI = 3.14159265;
					x = static_cast<float>(std::cos(rotX * PI / 180) * x - std::sin(rotX * PI / 180) * y);
					y = static_cast<float>(std::sin(rotX * PI / 180) * x + std::cos(rotX * PI / 180) * y);

					x += 0.5;
					y += 0.5;

					wrldSrc.push_back(cv::Point3f(x * 100,
						y * 100,
						z * 100));
				}
			}
		}

		if (wrldSrc.size() > 0) {
			std::vector<cv::Point2f> projected = projectPoints(wrldSrc);
			//std::vector<cv::Point2f> projected = (wrldSrc);
			for (int i = 0; i < projected.size(); i++)
			{
				if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {

					cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
					ROI.setTo(cv::Scalar(100, 100, 150, 100));
				}
			}
			if (plnSrc.size() > 0) {
				//projected = projectPoints(plnSrc);
				projected = projectPoints(plnSrc);
				for (int i = 0; i < projected.size(); i++)
				{
					if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {

						cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
						ROI.setTo(cv::Scalar(250, 100, 100, 100));
					}
				}
			}
			if (!gpuView) imshow("reprojection", board);
			else {
				libfreenect2::Frame b(640, 480, 4);
				b.data = board.data;
				viewer.addFrame("RGB", &b);
				shutdown = shutdown || viewer.render();
			}
		}
		(_listener)->release(frames);
		if (!gpuView) {
			int op = cv::waitKey(50);
			if (op == 100 || (char)(op) == 'd') right -= 1;
			if (op == 115 || (char)(op) == 's') up += 1;
			if (op == 97 || (char)(op) == 'a') right += 1;
			if (op == 119 || (char)(op) == 'w') up -= 1;

			if (op == 114 || (char)(op) == 'r') rotX -= 0.5;
			if (op == 102 || (char)(op) == 'f') rotX += 0.5;

			if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
			{
				std::cout << "right = " << right << ";\nup = " << up << ";\nrotX = " << rotX << ";\n";
				break;
			}
		}
		else {
			/*right = viewer.offsetX;
			up = viewer.offsetY;
			rotX = viewer.rot;*/
			right = 0;
			up = 0;
			rotX = 0;
		}
	}
	if (!gpuView) cv::destroyWindow("reprojection");
	else {
		viewer.stopWindow();
	}
}




std::vector<cv::Point3f> Projector::findPlane(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered) //RANSAC thing
{
	std::vector<cv::Point3f> wrldSrc;
	std::queue<cv::Point3f> queue;
	bool visited[513][425];
	for (int i = 0; i <= 512; i++)
		for (int j = 0; j <= 424; j++)
			visited[i][j] = false;
	cv::Vec3f f1, f2;
	float x1 = 0, y1 = 0, z1 = 0, color = 0;
	registration->getPointXYZRGB(undistorted, registered, 256, 213, x1, y1, z1, color);
	float x2 = 0, y2 = 0, z2 = 0;
	registration->getPointXYZRGB(undistorted, registered, 255, 212, x2, y2, z2, color);
	float x3 = 0, y3 = 0, z3 = 0;
	registration->getPointXYZRGB(undistorted, registered, 254, 214, x3, y3, z3, color);
	x1 *= 100; y1 *= 100; z1 *= 100;
	x2 *= 100; y2 *= 100; z2 *= 100;
	x3 *= 100; y3 *= 100; z3 *= 100;
	f1 = cv::Vec3f(x2 - x1, y2 - y1, z2 - z1);
	f2 = cv::Vec3f(x3 - x1, y3 - y1, z3 - z1);
	cv::Vec3f n = f1.cross(f2);
	//cv::normalize(n);
	double a, b, c, d;
	a = n[0];
	b = n[1];
	c = n[2];
	d = -(a*x1 + b*y1 + c*z1);
	for (int i = 0; i<512; i++)
	{
		for (int j = 0; j<424; j++)
		{
			float x = 0, y = 0, z = 0, color = 0;
			registration->getPointXYZRGB(undistorted, registered,
				i, j,
				x, y, z, color);

			if (z > 0.5 && z < 0.95)
			{
				x = static_cast<float>(x + right / ((double)640.0)); //////////TO-DO fix that
				y = static_cast<float>(y + up / ((double)480.0));

				x -= 0.5;
				y -= 0.5;
				double PI = 3.14159265;
				x = static_cast<float>(std::cos(rotX * PI / 180) * x - std::sin(rotX * PI / 180) * y);
				y = static_cast<float>(std::sin(rotX * PI / 180) * x + std::cos(rotX * PI / 180) * y);

				x += 0.5;
				y += 0.5;
				if (x*a*100 + y*b*100 + z*c*100 + d < 0.00001)
					wrldSrc.push_back(cv::Point3f(x * 100,
													y * 100,
													z * 100));
			}
		}
	}

	return wrldSrc;
}

std::vector<cv::Point2f> Projector::projectPoints(std::vector<cv::Point3f> in)
{
	std::vector<cv::Point2f> out;
	cv::projectPoints(in, _mr, _mt, _cam, _pro, out);
	return out;
}
