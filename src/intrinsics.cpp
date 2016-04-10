#include "intrinsics.h"



void Intrinsics::setup(Mat cameraMatrix, cv::Size imageSize, cv::Size sensorSize) {
		this->cameraMatrix = cameraMatrix;
		this->imageSize = imageSize;
		this->sensorSize = sensorSize;
		
        //Computes useful camera characteristics (fov, focalLength, principalPoint and aspectRatio) from the 
        // camera matrix, the camera frame resolution and the physical sensor size:
		calibrationMatrixValues(cameraMatrix, imageSize, sensorSize.width, sensorSize.height,
                                fov.x, fov.y, focalLength, principalPoint, aspectRatio);
        
	}
	
	Mat Intrinsics::getCameraMatrix() const {
		return cameraMatrix;
	}
	
	cv::Size Intrinsics::getImageSize() const {
		return imageSize;
	}
	
	cv::Size Intrinsics::getSensorSize() const {
		return sensorSize;
	}
	
	cv::Point2d Intrinsics::getFov() const {
		return fov;
	}
	
	double Intrinsics::getFocalLength() const {
		return focalLength;
	}
	
	double Intrinsics::getAspectRatio() const {
		return aspectRatio;
	}
	
	Point2d Intrinsics::getPrincipalPoint() const {
		return principalPoint;
	}




