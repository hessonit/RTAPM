#include "Projector.h"
#include "CTObject.h"
#include "objObject.h"
#include "util.h"
#include "viewer.h"

#include <iostream>
#include <queue>
#include <cmath>
#include <thread>

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
	ctProjection(ctFilePath, 1, xDim, yDim, zDim, false);
}

void Projector::ctProjection(std::string ctFilePath, int xDim, int yDim, int zDim, bool PNG)
{
	ctProjection(ctFilePath, 1, xDim, yDim, zDim, PNG);
}

void tst(libfreenect2::Registration* registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered,
	int st, int ed, cv::Mat *board, CTObject *ctObject, int right, int up,
	cv::Mat _mt, cv::Mat _mr, cv::Mat _cam, cv::Mat _pro, cv::Mat *depthFrame, cv::Mat *temp_board, 
	int *minX, int *maxX, int *minY, int *maxY) {
	float virtualVolumeSpace[6] = {-0.2f, 0.2f, -0.2f, 0.2f, 0.5f, 1.0f}; // tO-DO fix that
	int xDiv = (512 / abs(virtualVolumeSpace[0] - virtualVolumeSpace[1]));
	int yDiv = (512 / abs(virtualVolumeSpace[2] - virtualVolumeSpace[3]));
	int zDiv = (233 / abs(virtualVolumeSpace[4] - virtualVolumeSpace[5]));
	double xRight = right / ((double)640.0);
	double yUp = up / ((double)480.0);
	for (int i = st; i < ed; i++)
	{
		for (int j = 0; j < 424; j++)
		{
			float x = 0, y = 0, z = 0, color = 0;
			/*registration->getPointXYZRGB(undistorted, registered,
				j, i,
				x, y, z, color);*/
			registration->getPointXYZ(undistorted,
				j, i,
				x, y, z);
			//z = depthFrame->at<float>(j, i);

			if (z >= virtualVolumeSpace[4] && z <= virtualVolumeSpace[5] &&
				x >= virtualVolumeSpace[0] && x <= virtualVolumeSpace[1] &&
				y >= virtualVolumeSpace[2] && y <= virtualVolumeSpace[3]) {

				int xVal = (x + abs(virtualVolumeSpace[0]))*xDiv;
				int yVal = (y + abs(virtualVolumeSpace[2]))*yDiv;
				int zVal = (z - abs(virtualVolumeSpace[4]))*zDiv;

				x = static_cast<float>(x + xRight);
				y = static_cast<float>(y + yUp);

				std::vector<cv::Point3f> wrldSrc;
				wrldSrc.push_back(cv::Point3f(x, y, z) * 100);
				std::vector<cv::Point2f> projected;
				cv::projectPoints(wrldSrc, _mt, _mr, _cam, _pro, projected);

				if (480 - projected[0].x >0 && projected[0].y > 0 &&
					480 - projected[0].x < 475 && projected[0].y < 630) {
					temp_board->at<cv::Vec3s>(static_cast<int>(480 - projected[0].x), static_cast<int>(projected[0].y))[0] = (short)xVal;
					temp_board->at<cv::Vec3s>(static_cast<int>(480 - projected[0].x), static_cast<int>(projected[0].y))[1] = (short)yVal;
					temp_board->at<cv::Vec3s>(static_cast<int>(480 - projected[0].x), static_cast<int>(projected[0].y))[2] = (short)zVal;
					if (480 - projected[0].x < *minX) *minX = 480 - projected[0].x;
					if (480 - projected[0].x > *maxX) *maxX = 480 - projected[0].x;
					if (projected[0].y < *minY) *minY = projected[0].y;
					if (projected[0].y > *maxY) *maxY = projected[0].y;


				}
				//cv::Mat ROI = board(cv::Rect(i, j, 1, 1));
				//char value = ctObject.at(xVal, yVal, zVal);
				//ROI.setTo(cv::Scalar(value, value, value, 100));
			}	
		}
	}
	



}

cv::Mat pseudoBoxFilter(cv::Mat in, cv::Mat out, cv::Size ksize)
{
	cv::Mat result = cv::Mat(in);
	cv::Mat div(480, 640, CV_8UC1, cv::Scalar::all(0));
	int h = ksize.height;
	int w = ksize.width;
	int pow[10] = { 0,1,4,9,16,25,36,49,64,81 };
	for (int i = h / 2; i < in.size[0] - h / 2; i++)
	{
		for (int j = w / 2; j < in.size[1] - w / 2; j++)
		{
			cv::Vec3s v = in.at<cv::Vec3s>(i, j);
			if (v[0] != 0 && v[1] != 0 && v[2] != 0 && v[1])
			{
				for (int ii = i - h / 2; ii < i + h / 2; ii++)
				{
					for (int jj = j - w / 2; jj < j + w / 2; jj++)
					{
						cv::Vec3s vv = in.at<cv::Vec3s>(ii, jj);
						if (abs(ii - i)+abs(jj - j)<=h/2+1 && vv[0] == 0 && vv[1] == 0 && vv[2] == 0)
						{
							result.at<cv::Vec3s>(ii, jj)[0] += v[0] * pow[((h/2) - max(abs(ii - i), abs(jj - j))+1)];
							result.at<cv::Vec3s>(ii, jj)[1] += v[1] * pow[((h/2) - max(abs(ii - i), abs(jj - j))+1)];
							result.at<cv::Vec3s>(ii, jj)[2] += v[2] * pow[((h/2) - max(abs(ii - i), abs(jj - j))+1)];
							div.at<char>(ii, jj) += pow[((h / 2) - max(abs(ii - i), abs(jj - j)) +1)];
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < in.size[0]; i++)
	{
		for (int j = 0; j < in.size[1]; j++)
		{
			if(div.at<char>(i, j) != 0)
				result.at<cv::Vec3s>(i, j) /= div.at<char>(i, j);
		}
	}
	//out = result;
	return result;
}


void tt2(cv::Mat *board, int start, int end, cv::Size ksize, int minX, int maxX, int minY, int maxY)
{
	cv::Mat result = cv::Mat(*board);
	cv::Mat div(480, 640, CV_8UC1, cv::Scalar::all(0));
	int h = ksize.height;
	int w = ksize.width;
	int stX = max(max(start, h / 2), minX);
	int edX = min(min(end, board->size[0] - h / 2), maxX);

	int stY = max(w / 2, minY);
	int edY = min(board->size[1] - w / 2, maxY);

	int pow[10] = { 0,1,4,9,16,25,36,49,64,81 };
	for (int i = stX; i < edX; i++)
	{
		for (int j = stY; j < edY; j++)
		{
			cv::Vec3s v = board->at<cv::Vec3s>(i, j);
			if (v[0] != 0 && v[1] != 0 && v[2] != 0 && v[1])
			{
				for (int ii = i - h / 2; ii < i + h / 2; ii++)
				{
					for (int jj = j - w / 2; jj < j + w / 2; jj++)
					{
						cv::Vec3s vv = board->at<cv::Vec3s>(ii, jj);
						if (abs(ii - i) + abs(jj - j) <= h / 2 + 1 && vv[0] == 0 && vv[1] == 0 && vv[2] == 0)
						{
							result.at<cv::Vec3s>(ii, jj)[0] += v[0] * pow[((h / 2) - max(abs(ii - i), abs(jj - j)) + 1)];
							result.at<cv::Vec3s>(ii, jj)[1] += v[1] * pow[((h / 2) - max(abs(ii - i), abs(jj - j)) + 1)];
							result.at<cv::Vec3s>(ii, jj)[2] += v[2] * pow[((h / 2) - max(abs(ii - i), abs(jj - j)) + 1)];
							div.at<char>(ii, jj) += pow[((h / 2) - max(abs(ii - i), abs(jj - j)) + 1)];
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < board->size[0]; i++)
	{
		for (int j = 0; j < board->size[1]; j++)
		{
			if (div.at<char>(i, j) != 0)
				board->at<cv::Vec3s>(i, j) = result.at<cv::Vec3s>(i, j) / div.at<char>(i, j);
		}
	}

}

void tt3(cv::Mat *board, cv::Mat *temp_board, CTObject *ctObject, int start, int end)
{
	for (int i = start; i < end; i++)
	{
		for (int j = 0; j < 640; j++)
		{
			//cv::Mat ROI = (*board)(cv::Rect(j, i, 1, 1));
			cv::Vec4b v2 = board->at<cv::Vec4b>(i, j);
			cv::Vec3s v = temp_board->at<cv::Vec3s>(i, j);
			unsigned char value = ctObject->at(v[0], v[1], v[2]);
			v2[0] = v2[1] = v2[2] = value;
			//v2[1] = value;
			//v2[1] = value;
			//ROI.setTo(cv::Scalar(value, value, value, 100));
		}
	}
}

void loadNextFrame(libfreenect2::SyncMultiFrameListener *_listener, libfreenect2::FrameMap *frames, libfreenect2::Frame *rgb, libfreenect2::Frame *depth,
	libfreenect2::Registration* registration, libfreenect2::Frame *undistorted)
{
	_listener->release(*frames);
	//libfreenect2::FrameMap frames;
	(_listener)->waitForNewFrame(*frames);
	rgb = (*frames)[libfreenect2::Frame::Color];
	depth = (*frames)[libfreenect2::Frame::Depth];
	registration->undistortDepth(depth, undistorted);
	//registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);
}
void Projector::ctProjection(std::string ctFilePath, int startPoint, int xDim, int yDim, int zDim, bool PNG)
{
	SimpleViewer viewer;
	CTObject ctObject(ctFilePath, xDim, yDim, zDim, startPoint);
	if(PNG)
		ctObject.readDataPNG();
	else
		ctObject.readData();
	libfreenect2::FrameMap frames;
	libfreenect2::Registration* registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	bool shutdown = false;
	cv::Mat board(480, 640, CV_8UC4, cv::Scalar::all(0));
	//cv::namedWindow("CTviewer", CV_WINDOW_NORMAL);
	//cv::moveWindow("CTviewer", 00, 0);
	//cv::setWindowProperty("CTviewer", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	{
		viewer.setSize(480, 640); // TO-DO change resolution
		viewer.initialize();
		libfreenect2::Frame b(640, 480, 4);
		b.data = board.data;
		viewer.addFrame("RGB", &b);
		shutdown = shutdown || viewer.render();
	}
	libfreenect2::Frame *rgb;
	libfreenect2::Frame *depth;
	
	(_listener)->waitForNewFrame(frames);
	rgb = frames[libfreenect2::Frame::Color];
	depth = frames[libfreenect2::Frame::Depth];
	registration->undistortDepth(depth, &undistorted);

	while (!shutdown)
	{
		std::thread *nextFrame = new std::thread[1];
		/*(_listener)->waitForNewFrame(frames);
		rgb = frames[libfreenect2::Frame::Color];
		depth = frames[libfreenect2::Frame::Depth];

		registration->undistortDepth(depth, &undistorted);*/
		//registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		//cv::Mat frame = frameToMat("registered", &registered);
		cv::Mat depthFrame;// = frameToMat("depth", depth);
		//cv::Mat board(424, 512, CV_8UC4, cv::Scalar::all(0));
		board = cv::Mat(480, 640, CV_8UC4, cv::Scalar::all(0));

		//depthFrame *= 0.001;
		cv::Mat temp_board(480, 640, CV_16UC3, cv::Scalar::all(0));

		int parts = 128;
		int jump = 512 / parts;
		
		int minX = 999;
		int maxX = -1;
		int minY = 999;
		int maxY = -1;
		std::thread *tt = new std::thread[parts];

		for (int i = 0; i < parts; ++i) {
			tt[i] = std::thread(tst, registration, &undistorted, &registered, jump * i, jump + jump * i, &board, &ctObject,
				right, up, _mr, _mt, _cam, _pro, &depthFrame, &temp_board, &minX, &maxX, &minY, &maxY);
		}
		
		for (int i = 0; i < parts; ++i)
			tt[i].join();
		//_listener->release(frames);
		nextFrame[0] = std::thread(loadNextFrame, _listener, &frames, rgb, depth, registration, &undistorted);

		int parts2 = 8;
		int jump2 = 512 / parts2;
		for (int ii = 0; ii < 3; ii++) {
			for (int i = 0; i < parts2; ++i) {
				tt[i] = std::thread(tt2, &temp_board, jump2 * i, jump2 + jump2 * i, cv::Size(3, 3), minX, maxX, minY, maxY);
			}

			for (int i = 0; i < parts2; ++i)
				tt[i].join();
		}

		
		//for (int i = 0; i < 3; i++)
			//temp_board = pseudoBoxFilter(temp_board, temp_board, cv::Size(3, 3));

		//temp_board = pseudoBoxFilter(temp_board, temp_board, cv::Size(3, 3));
		//temp_board = pseudoBoxFilter(temp_board, temp_board, cv::Size(7, 7));
		//temp_board = pseudoBoxFilter(temp_board, temp_board, cv::Size(7, 7));
		
		//cv::medianBlur(temp_board, temp_board, 5); // TO-DO check if needed
		//cv::GaussianBlur(temp_board, temp_board, cv::Size(3, 3), 0, 1);
		//int parts3 = 16;
		//int jump3 = 480 / parts3;
		//std::thread *tt2 = new std::thread[parts];
		//for (int i = 0; i < parts3; ++i) {
		//	tt2[i] = std::thread(tt3, &board, &temp_board, &ctObject, jump3 * i, jump3 + jump3 * i);
		//	//std::cout << jump3 * i << " " << jump3 + jump3 * i << "\n";
		//}

		//for (int i = 0; i < parts; ++i)
		//	tt2[i].join();
		//std::cout << "zuoooooo\n";
		/*for (int i = 0; i < 480; i++)
		{
			for (int j = 0; j < 640; j++)
			{
				cv::Mat ROI = board(cv::Rect(j, i, 1, 1));
				cv::Vec3s v = temp_board.at<cv::Vec3s>(i, j);
				unsigned char value = ctObject.at(v[0], v[1], v[2]);
				ROI.setTo(cv::Scalar(value, value, value, 100));
			}
		}*/

		
		//cv::imshow("CTviewer", board);
		
		{
			libfreenect2::Frame b(640, 480, 4);
			b.data = board.data;
			viewer.addFrame("RGB", &b);
			shutdown = shutdown || viewer.render();
		}

		//_listener->release(frames);
		nextFrame[0].join();
		delete[] tt;
		delete[] nextFrame;
		
		//delete[] tt2;
		/*{
			int op = cv::waitKey(1);
			if (op == 100 || (char)(op) == 'd') right -= 1;
			if (op == 115 || (char)(op) == 's') up += 1;
			if (op == 97 || (char)(op) == 'a') right += 1;
			if (op == 119 || (char)(op) == 'w') up -= 1;

			if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
			{
				std::cout << "right = " << right << ";\nup = " << up << ";\nrotX = " << rotX << ";\n";
				shutdown = true;
				cv::destroyWindow("CTviewer");
			}
		}*/
	}


}

void Projector::objProjection(std::string objPath, std::string objName)
{
	std::cout << "Camera init: ";
	objObject obj(objPath, objName);
	obj.loadData();
	cout << "DONE\n";
	//cv::namedWindow("objTest", CV_WINDOW_NORMAL);
	//cv::moveWindow("objTest", 0, 0);
	indices.resize(480);
	for (int i = 0; i < 480; i++) indices[i].resize(640);

	libfreenect2::Registration* registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	libfreenect2::FrameMap frames;
	SimpleViewer viewer;
	bool shutdown = false;
	cv::Mat board(480, 640, CV_8UC4, cv::Scalar::all(255));
	cv::Vec3f prevNormal(-1, -1, -1);
		
		cv::namedWindow("reprojection", CV_WINDOW_NORMAL);
		cv::moveWindow("reprojection", 200, 200);
		//setWindowProperty("reprojection", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	while (!shutdown)
	{
		board = cv::Mat(480, 640, CV_8UC4, cv::Scalar::all(255));
		std::vector<cv::Point3f> plnSrc;
		cv::imshow("reprojection", board);
		(_listener)->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		PlaneData pln = findRectangle(registration, &undistorted, &registered);

		if (pln.points.size() > 0) {
			std::vector<cv::Point2f> projected = projectPoints(pln.points);
			cv::Mat cont = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(0));

			for (int i = 0; i < projected.size(); i++)
			{
				if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {

					cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
					ROI.setTo(cv::Scalar(250, 100, 100, 100));
					cont.at<uchar>(static_cast<int>(480 - projected[i].x), static_cast<int>(projected[i].y), 0) = 255;
					plnSrc.push_back(pln.points[i]);
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
				cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 8, true);
			}

				//drawContours(board, contours_poly, 0, cv::Scalar(0, 0, 0), 5, -1);
			cv::fillConvexPoly(board, contours_poly[0], cv::Scalar(0, 0, 0));

			cv::Mat data_pts = cv::Mat(300, 3, CV_64FC1);
			cv::Vec3f normal(0, 0, 0);
			int jump = plnSrc.size() / 300;
			for (int i = 0; i < 100; i++) {
				data_pts.at<double>(i, 0) = plnSrc[i*jump].x;
				data_pts.at<double>(i, 1) = plnSrc[i*jump].y;
				data_pts.at<double>(i, 2) = plnSrc[i*jump].z;
				data_pts.at<double>(i + 100, 0) = plnSrc[(i + 100)*jump].x;
				data_pts.at<double>(i + 100, 1) = plnSrc[(i + 100)*jump].y;
				data_pts.at<double>(i + 100, 2) = plnSrc[(i + 100)*jump].z;
				data_pts.at<double>(i + 200, 0) = plnSrc[(i + 200) *jump].x;
				data_pts.at<double>(i + 200, 1) = plnSrc[(i + 200)*jump].y;
				data_pts.at<double>(i + 200, 2) = plnSrc[(i + 200)*jump].z;
			}

			cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
			cv::Vec3f cntr = cv::Vec3f((pca_analysis.mean.at<double>(0, 0)),
				(pca_analysis.mean.at<double>(0, 1)),
				(pca_analysis.mean.at<double>(0, 2)));

			vector<cv::Point3f> eigen_vecs(2);
			vector<double> eigen_val(2);
			for (int i = 0; i < 2; ++i)
			{
				eigen_vecs[i] = cv::Point3f(pca_analysis.eigenvectors.at<double>(i, 0),
					pca_analysis.eigenvectors.at<double>(i, 1),
					pca_analysis.eigenvectors.at<double>(i, 2));
				eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
			}
			cv::Vec3f p1 = cv::Vec3f((eigen_vecs[0].x * eigen_val[0]), (eigen_vecs[0].y * eigen_val[0]), (eigen_vecs[0].z * eigen_val[0]));
			cv::Vec3f p2 = cv::Vec3f((eigen_vecs[1].x * eigen_val[1]), (eigen_vecs[1].y * eigen_val[1]), (eigen_vecs[1].z * eigen_val[1]));
			normal = p1.cross(p2);
			normal = cv::normalize(normal);

			pln.normal = normal;
			obj.setCamera(cv::Point3f(pln.center.x, -pln.center.y, -pln.center.z + 150),
				cv::Vec3f(pln.normal[0], pln.normal[1], pln.normal[2]));

			int maxX = -1, minX = 99999999;
			int maxY = -1, minY = 99999999;

			for (int i = 0; i < contours_poly[0].size(); i++) {
				if (contours_poly[0][i].x > maxX) maxX = contours_poly[0][i].x;
				if (contours_poly[0][i].x < minX) minX = contours_poly[0][i].x;
				if (contours_poly[0][i].y > maxY) maxY = contours_poly[0][i].y;
				if (contours_poly[0][i].y < minY) minY = contours_poly[0][i].y;
			}

			cv::Mat im = obj.render();
			cv::resize(im, im, cv::Size(maxX - minX, maxY - minY));
			cv::Mat rect = board(cv::Rect(minX, minY, maxX - minX, maxY - minY ));
			for (int i = 0; i < maxX - minX; i++)
			{
				for (int j = 0; j < maxY - minY; j++)
				{
					cv::Vec4b c = rect.at<cv::Vec4b>(j, i);
					if (c[2] == 0 )
					{
						cv::Vec3b b = im.at<cv::Vec3b>(j, i);
						cv::Vec4b col = cv::Vec4b(b[0], b[1], b[2], 255);
						rect.at<cv::Vec4b>(j, i) = col;
					}
				}
			}
			board(cv::Rect(minX, minY, maxX - minX, maxY - minY)) = rect;

			imshow("reprojection", board);
		
		}
		(_listener)->release(frames);
		{
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
	}
	cv::destroyWindow("reprojection");
}


void Projector::objProjectionOffline(std::string objPath, std::string objName, bool gpuView)
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
	cv::Vec3f prevNormal(-1, -1, -1);
	if (!gpuView) {
		cv::namedWindow("reprojection", CV_WINDOW_NORMAL);
		cv::moveWindow("reprojection", 200, 200);
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
		std::vector<cv::Point3f> plnSrc;
		if (!gpuView) cv::imshow("reprojection", board);
		(_listener)->waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		registration->apply(rgb, depth, &undistorted, &registered, true, NULL, NULL);

		PlaneData pln = findRectangle(registration, &undistorted, &registered);

		if (pln.points.size() > 0) {
			std::vector<cv::Point2f> projected = projectPoints(pln.points);
			cv::Mat cont = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(0));

			for (int i = 0; i < projected.size(); i++)
			{
				if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {

					cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
					ROI.setTo(cv::Scalar(250, 100, 100, 100));
					cont.at<uchar>(static_cast<int>(480 - projected[i].x), static_cast<int>(projected[i].y), 0) = 255;
					plnSrc.push_back(pln.points[i]);
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
			}

			for (int i = 0; i < contours.size(); i++)
			{
				drawContours(board, contours_poly, 0, cv::Scalar(0, 255, 0), 5);
			}


			cv::Mat data_pts = cv::Mat(300, 3, CV_64FC1);
			cv::Vec3f normal(0, 0, 0);
			int jump = plnSrc.size() / 300;
			for (int i = 0; i < 100; i++) {
				data_pts.at<double>(i, 0) = plnSrc[i*jump].x;
				data_pts.at<double>(i, 1) = plnSrc[i*jump].y;
				data_pts.at<double>(i, 2) = plnSrc[i*jump].z;
				data_pts.at<double>(i + 100, 0) = plnSrc[(i + 100)*jump].x;
				data_pts.at<double>(i + 100, 1) = plnSrc[(i + 100)*jump].y;
				data_pts.at<double>(i + 100, 2) = plnSrc[(i + 100)*jump].z;
				data_pts.at<double>(i + 200, 0) = plnSrc[(i + 200) *jump].x;
				data_pts.at<double>(i + 200, 1) = plnSrc[(i + 200)*jump].y;
				data_pts.at<double>(i + 200, 2) = plnSrc[(i + 200)*jump].z;
			}

			cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
			cv::Vec3f cntr = cv::Vec3f((pca_analysis.mean.at<double>(0, 0)),
				(pca_analysis.mean.at<double>(0, 1)),
				(pca_analysis.mean.at<double>(0, 2)));

			vector<cv::Point3f> eigen_vecs(2);
			vector<double> eigen_val(2);
			for (int i = 0; i < 2; ++i)
			{
				eigen_vecs[i] = cv::Point3f(pca_analysis.eigenvectors.at<double>(i, 0),
					pca_analysis.eigenvectors.at<double>(i, 1),
					pca_analysis.eigenvectors.at<double>(i, 2));
				eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
			}
			cv::Vec3f p1 = cv::Vec3f((eigen_vecs[0].x * eigen_val[0]), (eigen_vecs[0].y * eigen_val[0]), (eigen_vecs[0].z * eigen_val[0]));
			cv::Vec3f p2 = cv::Vec3f((eigen_vecs[1].x * eigen_val[1]), (eigen_vecs[1].y * eigen_val[1]), (eigen_vecs[1].z * eigen_val[1]));
			normal = p1.cross(p2);
			normal = cv::normalize(normal);
			//pln.center = cntr;

			pln.normal = normal;
			obj.setCamera(cv::Point3f(pln.center.x, -pln.center.y, -pln.center.z + 150),
				cv::Vec3f(pln.normal[0], pln.normal[1], pln.normal[2]));

			if (!gpuView) imshow("reprojection", board);
			else {
				libfreenect2::Frame b(640, 480, 4);
				b.data = board.data;
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
		cv::moveWindow("reprojection", 1200, 0);
		cv::setWindowProperty("reprojection", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
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
		
		PlaneData pln = findRectangle(registration, &undistorted, &registered);
		if (wrldSrc.size() > 0) {
			std::vector<cv::Point2f> projected = projectPoints(wrldSrc);
			for (int i = 0; i < projected.size(); i++)
			{
				if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {

					cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
					ROI.setTo(cv::Scalar(100, 100, 150, 100));
				}
			}
			if (pln.points.size() > 0) {
				projected = projectPoints(pln.points);
				cv::Mat cont = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(0));

				for (int i = 0; i < projected.size(); i++)
				{
					if (480 - projected[i].x >0 && projected[i].y > 0 && 480 - projected[i].x < 475 && projected[i].y < 630) {
						
						cv::Mat ROI = board(cv::Rect(static_cast<int>(projected[i].y), static_cast<int>(480 - projected[i].x), 2, 2));
						ROI.setTo(cv::Scalar(250, 100, 100, 100));
						cont.at<uchar>(static_cast<int>(480 - projected[i].x), static_cast<int>(projected[i].y), 0) = 255;
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
				int nPoly;
				for (int i = 0; i < contours.size(); i++)
				{
					cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 10, true);
					nPoly = contours_poly[i].size();
					
				}

				for (int i = 0; i< contours.size(); i++)
				{
					drawContours(board, contours_poly, 0, cv::Scalar(0, 255, 0), 5);
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

void Projector::setVirtualVolumeSpace(float x1, float x2, float y1, float y2, float z1, float z2)
{
	virtualVolumeSpace[0] = x1;
	virtualVolumeSpace[1] = x2;
	virtualVolumeSpace[2] = y1;
	virtualVolumeSpace[3] = y2;
	virtualVolumeSpace[4] = z1;
	virtualVolumeSpace[5] = z2;
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

PlaneData Projector::findRectangle(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered) //RANSAC thing
{
	PlaneData result;
	const int numberOfPoints = 3;
	int startingPoints[numberOfPoints][2] = { {256,213},
								{ 256,313},
								{ 256,113} };

	for (int i = 0; i < numberOfPoints; i++)
	{
		//std::cout << "I: " << i << "\n";
		result = findRectangleAt(registration, undistorted, registered, startingPoints[i][0], startingPoints[i][1]);
		if (result.points.size() > 0) break;
	}
	return result;
}


PlaneData Projector::findRectangleAt(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered, int startX, int startY)
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
	registration->getPointXYZRGB(undistorted, registered, startX, startY, x1, y1, z1, color);
	queue.push(cv::Point3f(startX, startY, z1));
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
	if (wrldSrc.size() < 6000) wrldSrc.clear();
	result.addInput(wrldSrc);
	return result;
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

cv::Vec3f PlaneData::computeNormal(cv::Point3f a, cv::Point3f b, cv::Point3f c)
{
	cv::Vec3f normal(0, 0, 0);
	cv::Vec3f f1 = cv::Vec3f(b.x - a.x, b.y - a.y, b.z - a.z);
	cv::Vec3f f2 = cv::Vec3f(c.x - a.x, c.y - a.y, c.z - a.z);
	normal = f1.cross(f2);
	if (normal[1] < 0) normal = -normal;
	normal = cv::normalize(normal);
	return normal;
}
