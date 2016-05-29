#pragma once

#include <opencv2\core.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

class Projector
{
public:
	Projector() { right = -40; up = 89; rotX = -4; };

	void setMatrices(cv::Mat mt, cv::Mat mr, cv::Mat cam, cv::Mat pro);
	void setKinect(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::Freenect2Device *dev);
	void ctProjection(std::string ctFilePath, int xDim, int yDim, int zDim);
	void objProjection(std::string objPath);
	void reproject(bool gpuView = false);
	void showRectangle(bool gpuView = false);

	int right, up, rotX;

private:

	std::vector<cv::Point3f> findRectangle(libfreenect2::Registration* registration, libfreenect2::Frame *undistorted, libfreenect2::Frame *registered);
	std::vector<cv::Point2f> projectPoints(std::vector<cv::Point3f> in);

	cv::Mat _mt, _mr, _cam, _pro;
	libfreenect2::SyncMultiFrameListener *_listener;
	libfreenect2::Freenect2Device *_dev;

};
