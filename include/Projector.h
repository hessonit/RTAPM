#pragma once

#include <cmath>

#include <opencv2\core.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

class PlaneData
{
public:
	PlaneData() {};
	void addInput(std::vector<cv::Point3f> _points);
	//void findOrientation(std::vector<cv::Point2f> _projectedPoints);
	void findOrientation(cv::Point3f a, cv::Point3f b, cv::Point3f c);

	std::vector<cv::Point3f> points;
	//std::vector<cv::Point2f> projectedPoints;
	cv::Point3f center;
	cv::Vec3f normal;
	cv::Vec3f up;
	static cv::Vec3f computeNormal(cv::Point3f a, cv::Point3f b, cv::Point3f c);

};

class Projector
{
public:
	Projector() { right = -40; up = 89; rotX = -4; };

	void setMatrices(cv::Mat mt, cv::Mat mr, cv::Mat cam, cv::Mat pro);
	void setKinect(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::Freenect2Device *dev);
	void ctProjection(std::string ctFilePath, int xDim, int yDim, int zDim);
	void objProjection(std::string objPath, std::string objName);
	void objProjectionOffline(std::string objPath, std::string objName, bool gpuView);
	void reproject(bool gpuView = false);
	void showRectangle(bool gpuView = false);

	int right, up, rotX;

private:

	int findIndex(int x, int y);
	PlaneData findRectangle(libfreenect2::Registration* registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered);
	PlaneData findRectangleAt(libfreenect2::Registration* registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered, int startX, int startY);

	std::vector<cv::Point2f> projectPoints(std::vector<cv::Point3f> in);

	cv::Mat _mt, _mr, _cam, _pro;
	libfreenect2::SyncMultiFrameListener *_listener;
	libfreenect2::Freenect2Device *_dev;
	std::vector<std::vector<int> >indices;

};


