#include "Projector.h"
#include "CTObject.h"
#include "objObject.h"
#include "util.h"
#include "viewer.h"

#include <iostream>
#include <queue>
#include <cmath>

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

void Projector::objProjection(std::string objPath, std::string objName, bool gpuView)
{
	std::cout << "Camera init: ";
	objObject obj(objPath, objName);
	obj.loadData();
	cout << "DONE\n";
	cv::namedWindow("objTest", CV_WINDOW_NORMAL);
	cv::moveWindow("objTest", 0, 0);
	indices.resize(480);
	for (int i = 0; i < 480; i++) indices[i].resize(640);

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

		PlaneData pln = findRectangle(registration, &undistorted, &registered);
		//obj.setCamera(pln.center, cv::Vec3f(0, 0, 0));
		
		if (pln.points.size() > 0) {
			std::vector<cv::Point2f> projected = projectPoints(pln.points);
			//pln.projectedPoints = projected;
			cv::Mat cont = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(0));

			for (int i = 0; i < 480; i++)
				for (int j = 0; j < 640; j++)
					indices[i][j] = -1;

			cv::Point2d maxX(-1, 0);
			cv::Point2d maxY(0, -1);
			cv::Point2d minX(99999999, 0);
			cv::Point2d minY(0, 99999999);
			for (int i = 0; i < projected.size(); i++)
			{
				if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {

					cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
					ROI.setTo(cv::Scalar(250, 100, 100, 100));

					cont.at<uchar>(static_cast<int>(480 - projected[i].x), static_cast<int>(projected[i].y), 0) = 255;
					//std::cout << (int)(480 - projected[i].x) << " ";
					//indices[(int)(480 - projected[i].x)][0] = 123;
					indices[static_cast<int>(480 - projected[i].x)][static_cast<int>(projected[i].y)] = i;
				}
			}
			vector<vector<cv::Point> > contours;
			vector<cv::Vec4i> hierarchy;
			cv::GaussianBlur(cont, cont, cv::Size(7, 7), 5, 11);
			findContours(cont, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

			vector<vector<cv::Point> > contours_poly(contours.size());
			vector<cv::Rect> boundRect(contours.size());
			vector<cv::Point2f>center(contours.size());
			vector<float>radius(contours.size());

			for (int i = 0; i < contours.size(); i++)
			{
				cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 10, true);
				int nPoly = contours_poly[i].size();
				if (nPoly < 10 && nPoly > 2) {
					int a = findIndex(contours_poly[i][0].y, contours_poly[i][0].x);
					int b = findIndex(contours_poly[i][nPoly / 3].y, contours_poly[i][nPoly / 3].x);
					int c = findIndex(contours_poly[i][2 * nPoly / 3].y, contours_poly[i][2 * nPoly / 3].x);
					pln.findOrientation(pln.points[a], pln.points[b], pln.points[c]);
					//std::cout << pln.points[a] << " " << pln.points[b] << " " << pln.points[c] << "\n";
					//std::cout << "center: " << pln.center << "\n";
					std::cout << "normal: " << pln.normal << " up: " << pln.up << "\n";

					//std::cout << a << " " << b << " " << c << "\n";
					
				}
			}

			for (int i = 0; i < contours.size(); i++)
			{
				drawContours(board, contours_poly, 0, cv::Scalar(0, 255, 0), 5);
			}

			//obj.setCamera(cv::Point3f(pln.center.x, pln.center.y - 30, -pln.center.z + 150),
				//cv::Vec3f(pln.normal[0], pln.normal[1], pln.normal[2]));

			obj.setCamera(cv::Point3f(pln.center.x, -pln.center.y, -pln.center.z + 150),
				cv::Vec3f(pln.normal[0], pln.normal[1], pln.normal[2]));

			//imshow("reprojection", cont);
			//imshow("reprojection", board);


			if (!gpuView) imshow("reprojection", board);
			else {
				libfreenect2::Frame b(640, 480, 4);
				b.data = board.data;
				//b.data = rect.data;
				viewer.addFrame("RGB", &b);
				shutdown = shutdown || viewer.render();
			}
		}
		cv::Mat im = obj.render();
		cv::imshow("objTest", im);
		//}
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
			//right = 0;
			//up = 0;
			//rotX = 0;
			right = viewer.offsetX;
			up = viewer.offsetY;
			rotX = viewer.rot;
		}
	}
	if (!gpuView) cv::destroyWindow("reprojection");
	else {
		viewer.stopWindow();
	}
	cv::destroyWindow("objTest");
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

void Projector::showRectangle(bool gpuView)
{
	//int indices[480][640];
	/*vector<vector<int> >indices;*/
	indices.resize(480);
	for (int i = 0; i < 480; i++) indices[i].resize(640);

	libfreenect2::Registration* registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	libfreenect2::FrameMap frames;
	SimpleViewer viewer;
	bool shutdown = false;
	cv::Mat board(480, 640, CV_8UC4, cv::Scalar::all(255));
	//cv::Mat rect(480, 640, CV_8UC4, cv::Scalar::all(255));
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
		//rect = cv::Mat(480, 640, CV_8UC4, cv::Scalar::all(255));
		std::vector<cv::Point3f> wrldSrc;
		std::vector<cv::Point3f> plnSrc;
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

				if (z>0.5 && z<2.1)
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
		
		//plnSrc = findRectangle(registration, &undistorted, &registered);
		PlaneData pln = findRectangle(registration, &undistorted, &registered);
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
			//if (plnSrc.size() > 0) {
			if (pln.points.size() > 0) {
				//projected = projectPoints(plnSrc);
				projected = projectPoints(pln.points);
				//pln.projectedPoints = projected;
				cv::Mat cont = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(0));

				//int indices[480][640];
				for (int i = 0; i < 480; i++)
					for (int j = 0; j < 640; j++)
						indices[i][j] = -1;

				cv::Point2d maxX(-1, 0);
				cv::Point2d maxY(0, -1);
				cv::Point2d minX(99999999, 0);
				cv::Point2d minY(0, 99999999);
				for (int i = 0; i < projected.size(); i++)
				{
					if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {
						
						cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
						ROI.setTo(cv::Scalar(250, 100, 100, 100));

						cont.at<uchar>(static_cast<int>(480 - projected[i].x), static_cast<int>(projected[i].y), 0) = 255;
						//std::cout << (int)(480 - projected[i].x) << " ";
						//indices[(int)(480 - projected[i].x)][0] = 123;
						indices[static_cast<int>(480 - projected[i].x)][static_cast<int>(projected[i].y)] = i;
					}
				}
				vector<vector<cv::Point> > contours;
				vector<cv::Vec4i> hierarchy;
				cv::GaussianBlur(cont, cont, cv::Size(7, 7), 5, 11);
				
				//cv::blur(cont, cont, cv::Size(4, 4));
				//cv::threshold(cont, cont, 80, 255, cv::THRESH_BINARY);
				findContours(cont, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
				//drawContours(board, contours, 0, cv::Scalar(0, 255, 0), 5);
				//std::cout << "countours: " << contours[0].size() << "\n";


				vector<vector<cv::Point> > contours_poly(contours.size());
				vector<cv::Rect> boundRect(contours.size());
				vector<cv::Point2f>center(contours.size());
				vector<float>radius(contours.size());

				for (int i = 0; i < contours.size(); i++)
				{
					cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 10, true);
					int nPoly = contours_poly[i].size();
					if (nPoly < 10 && nPoly > 2) {
						int a = findIndex(contours_poly[i][0].y, contours_poly[i][0].x);
						int b = findIndex(contours_poly[i][nPoly/2].y, contours_poly[i][nPoly/2].x);
						int c = findIndex(contours_poly[i][2*nPoly/3].y, contours_poly[i][2*nPoly/3].x);

						//std::cout << a << " " << b << " " << c << "\n";
					}
				}

				for (int i = 0; i< contours.size(); i++)
				{
					drawContours(board, contours_poly, 0, cv::Scalar(0, 255, 0), 5);
				}

				
				//imshow("reprojection", cont);
				//imshow("reprojection", board);

			}
			if (!gpuView) imshow("reprojection", board);
			else {
				libfreenect2::Frame b(640, 480, 4);
				b.data = board.data;
				//b.data = rect.data;
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
			right = 0;
			up = 0;
			rotX = 0;
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



int Projector::findIndex(int x, int y)
{
	int result = -1;
	
	std::queue<cv::Point2d > queue;
	bool visited[480][640];
	for (int i = 0; i < 480; i++)
		for (int j = 0; j < 640; j++)
			visited[i][j] = false;

	queue.push(cv::Point2d(x, y));
	while (!queue.empty())
	{
		cv::Point2d p = queue.front();
		queue.pop();
		visited[(int)p.x][(int)p.y] = true;
		if (indices[p.x][p.y] != -1)
			return indices[p.x][p.y];
		if (p.x > 0 && !visited[(int)p.x - 1][(int)p.y]) queue.push(cv::Point2d(p.x - 1, p.y));
		if (p.x < 479 && !visited[(int)p.x + 1][(int)p.y]) queue.push(cv::Point2d(p.x + 1, p.y));
		if (p.y > 0 && !visited[(int)p.x][(int)p.y - 1]) queue.push(cv::Point2d(p.x, p.y - 1));
		if (p.x < 639 && !visited[(int)p.x][(int)p.y+1]) queue.push(cv::Point2d(p.x, p.y + 1));

	}

	return result;
}

// TODO? multiple starting points (now only in the middle)
//std::vector<cv::Point3f> Projector::findRectangle(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered) //RANSAC thing
PlaneData Projector::findRectangle(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered) //RANSAC thing
{
	PlaneData result;
	std::vector<cv::Point3f> wrldSrc;
	std::queue<cv::Point3f > queue;
	bool visited[513][425];
	for (int i = 0; i <= 512; i++)
		for (int j = 0; j <= 424; j++)
			visited[i][j] = false;
	double constant = 0.007;
	int num = 100;
	float x1 = 0, y1 = 0, z1 = 0, color = 0;
	registration->getPointXYZRGB(undistorted, registered, 256, 213, x1, y1, z1, color);
	queue.push(cv::Point3f(256, 213, z1));
	wrldSrc.push_back(cv::Point3f(x1, y1, z1)*num);
	while (!queue.empty())
	{
		cv::Point3f p = queue.front();
		queue.pop();
		if (!visited[(int)p.x][(int)p.y])
		{
			int ix = (int)p.x;
			int iy = (int)p.y;
			visited[ix][iy] = true;
			float x = 0, y = 0, z = 0, color = 0;
			if (ix < 511 && !visited[ix + 1][iy])
			{
				registration->getPointXYZRGB(undistorted, registered, ix + 1, iy, x, y, z, color);
				if (abs(z - p.z) < constant)
				{
					queue.push(cv::Point3f(ix + 1, iy, z));
					wrldSrc.push_back(cv::Point3f(x, y, z)*num);
				}
			}
			if (ix >  0 && !visited[ix - 1][iy])
			{
				registration->getPointXYZRGB(undistorted, registered, ix - 1, iy, x, y, z, color);
				if (abs(z - p.z) < constant)
				{
					queue.push(cv::Point3f(ix - 1, iy, z));
					wrldSrc.push_back(cv::Point3f(x, y, z)*num);
				}
			}
			if (iy < 423 && !visited[ix][iy + 1])
			{
				registration->getPointXYZRGB(undistorted, registered, ix, iy + 1, x, y, z, color);
				if (abs(z - p.z) < constant)
				{
					queue.push(cv::Point3f(ix, iy + 1, z));
					wrldSrc.push_back(cv::Point3f(x, y, z)*num);
				}
			}
			if (iy > 0 && !visited[ix][iy - 1])
			{
				registration->getPointXYZRGB(undistorted, registered, ix, iy - 1, x, y, z, color);
				if (abs(z - p.z) < constant)
				{
					queue.push(cv::Point3f(ix, iy - 1, z));
					wrldSrc.push_back(cv::Point3f(x, y, z)*num);
				}
			}

		}
	}
	if (wrldSrc.size() < 2500) wrldSrc.clear();
	result.addInput(wrldSrc);
	return result;
	//return wrldSrc;
}

std::vector<cv::Point2f> Projector::projectPoints(std::vector<cv::Point3f> in)
{
	std::vector<cv::Point2f> out;
	cv::projectPoints(in, _mr, _mt, _cam, _pro, out);
	return out;
} 

void PlaneData::addInput(std::vector<cv::Point3f> _points)
{
	points = _points;
	center = cv::Point3f(0, 0, 0);
	for (int i = 0; i < points.size(); i++)
	{
		center += points[i];
		/*center.x += points[i].x;
		center.y += points[i].y;
		center.z += points[i].z;*/
	}
	if(!points.empty())
		center = center / (double)points.size();
	//std::cout << "center point: " << center.x << " " << center.y << " " << center.z << "\n";
}

void PlaneData::findOrientation(cv::Point3f a, cv::Point3f b, cv::Point3f c)
{
	cv::Vec3f f1 = cv::Vec3f(b.x - a.x, b.y - a.y, b.z - a.z);
	cv::Vec3f f2 = cv::Vec3f(c.x - a.x, c.y - a.y, c.z - a.z);
	normal = f1.cross(f2);
	//std::cout << "f1:" << f1 << " f2:" << f2 << " no:" << normal << "\n";
	if (normal[1] < 0) normal = -normal;
	normal = cv::normalize(normal);
	cv::Vec3f tempUp(0, 0, 1);
	tempUp = normal.cross(tempUp);
	up = normal.cross(tempUp);
	up = cv::normalize(up);
	//std::cout << "tp:" << tempUp << " up:" << up << " no:" << normal << "\n";
}

//void PlaneData::findOrientation(std::vector<cv::Point2f> _projectedPoints)
//{
//	projectedPoints = _projectedPoints;
//}
